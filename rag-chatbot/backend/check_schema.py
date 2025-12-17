"""Check existing database schema."""

import asyncio
from sqlalchemy import text
from auth_backend.database import engine


async def check_schema():
    """Check columns in sessions table."""
    async with engine.connect() as conn:
        # Check if sessions table exists and what columns it has
        result = await conn.execute(
            text("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'sessions'
                ORDER BY ordinal_position;
            """)
        )
        columns = [(row[0], row[1]) for row in result]

        if columns:
            print("\n❗ Existing 'sessions' table columns:")
            for col_name, col_type in columns:
                print(f"   - {col_name} ({col_type})")
            print("\n⚠️  This table likely belongs to the RAG chatbot.")
            print("We need to use a different table name for auth sessions.\n")
        else:
            print("\n✅ No existing sessions table found.\n")


if __name__ == "__main__":
    asyncio.run(check_schema())
