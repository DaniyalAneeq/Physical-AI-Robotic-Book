"""Database migration script for creating sessions and query_logs tables."""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import psycopg
from src.config import get_settings

settings = get_settings()

MIGRATIONS = [
    # Create sessions table
    """
    CREATE TABLE IF NOT EXISTS sessions (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
        last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
        conversation_history JSONB DEFAULT '[]',
        scope_context JSONB DEFAULT '{}'
    );
    """,
    # Create index for session expiration cleanup
    """
    CREATE INDEX IF NOT EXISTS idx_sessions_last_activity
    ON sessions(last_activity);
    """,
    # Create query_logs table
    """
    CREATE TABLE IF NOT EXISTS query_logs (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        session_id UUID,
        query_text TEXT NOT NULL,
        scope_type VARCHAR(50),
        response_latency_ms INTEGER,
        chunk_count INTEGER,
        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
    );
    """,
    # Create index for analytics queries
    """
    CREATE INDEX IF NOT EXISTS idx_query_logs_created_at
    ON query_logs(created_at);
    """,
    # Create index for session-based log lookup
    """
    CREATE INDEX IF NOT EXISTS idx_query_logs_session_id
    ON query_logs(session_id);
    """,
]


async def run_migrations():
    """Run all database migrations."""
    print(f"Connecting to database...")

    async with await psycopg.AsyncConnection.connect(settings.neon_database_url) as conn:
        print("Connected successfully!")

        for i, migration in enumerate(MIGRATIONS, 1):
            print(f"Running migration {i}/{len(MIGRATIONS)}...")
            try:
                await conn.execute(migration)
                await conn.commit()
                print(f"  Migration {i} completed.")
            except Exception as e:
                print(f"  Migration {i} failed: {e}")
                # Continue with other migrations (IF NOT EXISTS makes them idempotent)

        print("\nVerifying tables...")

        # Verify sessions table
        async with conn.cursor() as cur:
            await cur.execute("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'sessions'
                ORDER BY ordinal_position;
            """)
            columns = await cur.fetchall()
            print("\nSessions table columns:")
            for col in columns:
                print(f"  - {col[0]}: {col[1]}")

        # Verify query_logs table
        async with conn.cursor() as cur:
            await cur.execute("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'query_logs'
                ORDER BY ordinal_position;
            """)
            columns = await cur.fetchall()
            print("\nQuery_logs table columns:")
            for col in columns:
                print(f"  - {col[0]}: {col[1]}")

        print("\nMigrations completed successfully!")


def main():
    """Entry point for migration script."""
    print("=" * 50)
    print("RAG Chatbot Database Migration")
    print("=" * 50)
    asyncio.run(run_migrations())


if __name__ == "__main__":
    main()
