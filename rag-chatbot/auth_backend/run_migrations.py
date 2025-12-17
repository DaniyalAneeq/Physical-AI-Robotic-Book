#!/usr/bin/env python3
"""
Simple migration runner to create all database tables.

This script creates all tables defined in the models if they don't exist.
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from auth_backend.database import engine
from auth_backend.models.base import Base
from auth_backend.models.user import User
from auth_backend.models.session import Session
from auth_backend.models.oauth_account import OAuthAccount
from auth_backend.models.onboarding_profile import OnboardingProfile


async def run_migrations():
    """Create all database tables."""
    print("🔄 Running migrations...")
    print(f"📊 Models loaded:")
    print(f"  - User")
    print(f"  - Session")
    print(f"  - OAuthAccount")
    print(f"  - OnboardingProfile")

    try:
        async with engine.begin() as conn:
            # Create all tables
            await conn.run_sync(Base.metadata.create_all)

        print("✅ All tables created successfully!")
        print("\n📋 Tables created:")
        for table in Base.metadata.sorted_tables:
            print(f"  - {table.name}")

        return True
    except Exception as e:
        print(f"❌ Error running migrations: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(run_migrations())
    sys.exit(0 if success else 1)
