"""Verify authentication tables exist."""

import asyncio
from sqlalchemy import text
from auth_backend.database import engine


async def verify_tables():
    """Check if all auth tables exist."""
    async with engine.connect() as conn:
        result = await conn.execute(
            text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name IN ('users', 'auth_sessions', 'oauth_accounts')
                ORDER BY table_name;
            """)
        )
        tables = [row[0] for row in result]

        print("\n✅ Authentication tables:")
        for table in tables:
            print(f"   ✓ {table}")

        if len(tables) == 3:
            print("\n🎉 All authentication tables ready!\n")
        else:
            print(f"\n⚠️  Expected 3 tables, found {len(tables)}\n")


if __name__ == "__main__":
    asyncio.run(verify_tables())
