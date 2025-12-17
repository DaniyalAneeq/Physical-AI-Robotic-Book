"""Quick script to verify database tables exist."""

import asyncio
from sqlalchemy import text
from auth_backend.database import engine


async def verify_tables():
    """Check if auth tables exist in database."""
    async with engine.connect() as conn:
        result = await conn.execute(
            text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name IN ('users', 'sessions', 'oauth_accounts')
                ORDER BY table_name;
            """)
        )
        tables = [row[0] for row in result]

        print("\n✅ Database tables verified:")
        for table in tables:
            print(f"   ✓ {table}")

        if len(tables) == 3:
            print("\n🎉 All authentication tables created successfully!\n")
            return True
        else:
            print(f"\n⚠️  Expected 3 tables, found {len(tables)}\n")
            return False


if __name__ == "__main__":
    asyncio.run(verify_tables())
