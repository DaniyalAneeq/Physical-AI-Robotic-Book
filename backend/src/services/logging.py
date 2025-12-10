"""Query logging service for analytics (no PII)."""

import json
from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4

from ..config import get_settings
from ..services.session import session_service

settings = get_settings()


class LoggingService:
    """Service for logging queries for analytics.

    Privacy Note: No PII is stored. Session IDs are random UUIDs
    not linked to user identity.
    """

    async def log_query(
        self,
        session_id: Optional[UUID],
        query_text: str,
        scope_type: str,
        response_latency_ms: int,
        chunk_count: int,
    ) -> UUID:
        """Log a query for analytics.

        Args:
            session_id: Session UUID (can be None for anonymous queries)
            query_text: User's question
            scope_type: Scope type used ('page', 'selection', 'module', 'all')
            response_latency_ms: End-to-end latency in milliseconds
            chunk_count: Number of chunks retrieved from Qdrant

        Returns:
            Log entry UUID
        """
        log_id = uuid4()

        async with session_service.pool.connection() as conn:
            await conn.execute(
                """
                INSERT INTO query_logs
                (id, session_id, query_text, scope_type, response_latency_ms, chunk_count, created_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                """,
                (
                    str(log_id),
                    str(session_id) if session_id else None,
                    query_text,
                    scope_type,
                    response_latency_ms,
                    chunk_count,
                    datetime.utcnow(),
                ),
            )

        return log_id

    async def get_stats(
        self,
        hours: int = 24,
    ) -> dict:
        """Get query statistics for the specified time period.

        Args:
            hours: Number of hours to look back

        Returns:
            Dictionary with statistics
        """
        async with session_service.pool.connection() as conn:
            async with conn.cursor() as cur:
                # Total queries
                await cur.execute(
                    """
                    SELECT COUNT(*) FROM query_logs
                    WHERE created_at > NOW() - INTERVAL '%s hours'
                    """,
                    (hours,),
                )
                total_queries = (await cur.fetchone())[0]

                # Average latency
                await cur.execute(
                    """
                    SELECT AVG(response_latency_ms),
                           PERCENTILE_CONT(0.95) WITHIN GROUP (ORDER BY response_latency_ms)
                    FROM query_logs
                    WHERE created_at > NOW() - INTERVAL '%s hours'
                    AND response_latency_ms IS NOT NULL
                    """,
                    (hours,),
                )
                row = await cur.fetchone()
                avg_latency = row[0]
                p95_latency = row[1]

                # Queries by scope type
                await cur.execute(
                    """
                    SELECT scope_type, COUNT(*)
                    FROM query_logs
                    WHERE created_at > NOW() - INTERVAL '%s hours'
                    GROUP BY scope_type
                    """,
                    (hours,),
                )
                scope_breakdown = dict(await cur.fetchall())

                # Unique sessions
                await cur.execute(
                    """
                    SELECT COUNT(DISTINCT session_id) FROM query_logs
                    WHERE created_at > NOW() - INTERVAL '%s hours'
                    AND session_id IS NOT NULL
                    """,
                    (hours,),
                )
                unique_sessions = (await cur.fetchone())[0]

        return {
            "period_hours": hours,
            "total_queries": total_queries,
            "avg_latency_ms": round(avg_latency, 2) if avg_latency else None,
            "p95_latency_ms": round(p95_latency, 2) if p95_latency else None,
            "scope_breakdown": scope_breakdown,
            "unique_sessions": unique_sessions,
        }

    async def cleanup_old_logs(self, days: int = 30) -> int:
        """Delete logs older than specified days.

        Args:
            days: Number of days to retain

        Returns:
            Number of logs deleted
        """
        async with session_service.pool.connection() as conn:
            result = await conn.execute(
                """
                DELETE FROM query_logs
                WHERE created_at < NOW() - INTERVAL '%s days'
                """,
                (days,),
            )
        return result.rowcount


# Singleton instance
logging_service = LoggingService()
