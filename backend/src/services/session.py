"""Session management service with Neon Postgres."""

import json
from datetime import datetime, timedelta
from typing import Optional
from uuid import UUID, uuid4

import psycopg
from psycopg.rows import dict_row
from psycopg_pool import AsyncConnectionPool

from ..config import get_settings
from ..models.session import Session, ScopeContext, ScopeType, ConversationEntry

settings = get_settings()


class SessionService:
    """Service for managing user sessions in Neon Postgres."""

    def __init__(self):
        """Initialize session service."""
        self._pool: Optional[AsyncConnectionPool] = None

    async def initialize(self) -> None:
        """Initialize the connection pool."""
        if self._pool is None:
            self._pool = AsyncConnectionPool(
                settings.neon_database_url,
                min_size=1,
                max_size=10,
                open=False,
            )
            await self._pool.open()

    async def close(self) -> None:
        """Close the connection pool."""
        if self._pool:
            await self._pool.close()
            self._pool = None

    @property
    def pool(self) -> AsyncConnectionPool:
        """Get the connection pool."""
        if self._pool is None:
            raise RuntimeError("SessionService not initialized. Call initialize() first.")
        return self._pool

    async def validate_connection(self) -> bool:
        """Validate connection to Postgres."""
        try:
            async with self.pool.connection() as conn:
                await conn.execute("SELECT 1")
            return True
        except Exception:
            return False

    async def create(self, scope_context: Optional[ScopeContext] = None) -> Session:
        """Create a new session.

        Args:
            scope_context: Optional initial scope context

        Returns:
            Created Session object
        """
        session_id = uuid4()
        now = datetime.utcnow()
        scope = scope_context or ScopeContext(type=ScopeType.ALL)

        async with self.pool.connection() as conn:
            await conn.execute(
                """
                INSERT INTO sessions (id, created_at, last_activity, conversation_history, scope_context)
                VALUES (%s, %s, %s, %s, %s)
                """,
                (
                    str(session_id),
                    now,
                    now,
                    json.dumps([]),
                    json.dumps(scope.model_dump()),
                ),
            )

        return Session(
            id=session_id,
            created_at=now,
            last_activity=now,
            conversation_history=[],
            scope_context=scope,
        )

    async def get(self, session_id: UUID) -> Optional[Session]:
        """Retrieve a session by ID.

        Args:
            session_id: Session UUID

        Returns:
            Session object if found, None otherwise
        """
        async with self.pool.connection() as conn:
            async with conn.cursor(row_factory=dict_row) as cur:
                await cur.execute(
                    """
                    SELECT id, created_at, last_activity, conversation_history, scope_context
                    FROM sessions
                    WHERE id = %s
                    """,
                    (str(session_id),),
                )
                row = await cur.fetchone()

        if not row:
            return None

        # Parse conversation history
        history_data = row["conversation_history"] or []
        if isinstance(history_data, str):
            history_data = json.loads(history_data)

        conversation_history = [
            ConversationEntry(**entry) for entry in history_data
        ]

        # Parse scope context
        scope_data = row["scope_context"] or {"type": "all"}
        if isinstance(scope_data, str):
            scope_data = json.loads(scope_data)
        scope_context = ScopeContext(**scope_data)

        return Session(
            id=UUID(row["id"]) if isinstance(row["id"], str) else row["id"],
            created_at=row["created_at"],
            last_activity=row["last_activity"],
            conversation_history=conversation_history,
            scope_context=scope_context,
        )

    async def update(
        self,
        session_id: UUID,
        query: str,
        response: str,
        scope_context: Optional[ScopeContext] = None,
    ) -> Optional[Session]:
        """Update session with new query/response pair.

        Args:
            session_id: Session UUID
            query: User's question
            response: Chatbot's answer
            scope_context: Optional updated scope context

        Returns:
            Updated Session object if found, None otherwise
        """
        session = await self.get(session_id)
        if not session:
            return None

        # Add new entry, respecting max history limit
        new_entry = ConversationEntry(
            query=query,
            response=response,
            timestamp=datetime.utcnow(),
        )

        history = session.conversation_history.copy()
        history.append(new_entry)

        # Enforce max history limit (FIFO)
        if len(history) > settings.max_conversation_history:
            history = history[-settings.max_conversation_history:]

        now = datetime.utcnow()
        scope = scope_context or session.scope_context

        async with self.pool.connection() as conn:
            await conn.execute(
                """
                UPDATE sessions
                SET last_activity = %s,
                    conversation_history = %s,
                    scope_context = %s
                WHERE id = %s
                """,
                (
                    now,
                    json.dumps([e.model_dump() for e in history], default=str),
                    json.dumps(scope.model_dump()),
                    str(session_id),
                ),
            )

        return Session(
            id=session_id,
            created_at=session.created_at,
            last_activity=now,
            conversation_history=history,
            scope_context=scope,
        )

    async def delete(self, session_id: UUID) -> bool:
        """Delete a session.

        Args:
            session_id: Session UUID

        Returns:
            True if deleted, False if not found
        """
        async with self.pool.connection() as conn:
            result = await conn.execute(
                "DELETE FROM sessions WHERE id = %s",
                (str(session_id),),
            )
        return result.rowcount > 0

    async def cleanup(self) -> int:
        """Delete sessions inactive for more than 24 hours.

        Returns:
            Number of sessions deleted
        """
        expiry_time = datetime.utcnow() - timedelta(hours=settings.session_expiry_hours)

        async with self.pool.connection() as conn:
            result = await conn.execute(
                "DELETE FROM sessions WHERE last_activity < %s",
                (expiry_time,),
            )
        return result.rowcount


# Singleton instance
session_service = SessionService()
