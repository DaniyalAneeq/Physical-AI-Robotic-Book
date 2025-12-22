#!/usr/bin/env python3
"""
⚠️  DEPRECATED - This script is no longer used.

The authentication backend has been merged with the RAG backend into a unified server.

To start the unified server, use:
    cd ../backend
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

The unified server serves both:
    - RAG endpoints: /api/*
    - Auth endpoints: /auth/*

For migrations, use:
    python -m auth_backend.migrations.001_initial_auth
    python -m auth_backend.migrations.002_onboarding

OLD DOCUMENTATION BELOW (for reference only):
================================================================

Quick start script for authentication backend.

Usage:
    python run.py                 # Run server (DEPRECATED)
    python run.py --test          # Run tests
    python run.py --migrate       # Run migrations
    python run.py --generate-secret  # Generate session secret
"""

import argparse
import asyncio
import secrets
import sys
from pathlib import Path


def generate_secret():
    """Generate a secure random session secret."""
    secret = secrets.token_urlsafe(32)
    print("\n" + "=" * 60)
    print("🔐 Generated Secure Session Secret:")
    print("=" * 60)
    print(f"\n{secret}\n")
    print("Add this to your .env file:")
    print(f"SESSION_SECRET={secret}")
    print("\n" + "=" * 60 + "\n")


async def run_migrations():
    """Run database migrations."""
    print("🔄 Running database migrations...")
    try:
        from auth_backend.database import engine
        from auth_backend.models.base import Base

        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

        print("✅ Database migrations completed successfully!")
        print("\nTables created:")
        print("  - users")
        print("  - sessions")
        print("  - oauth_accounts")
    except Exception as e:
        print(f"❌ Migration failed: {e}")
        sys.exit(1)


def run_tests():
    """Run the test suite."""
    print("🧪 Running tests...")
    import pytest

    exit_code = pytest.main([
        "auth_backend/tests/",
        "-v",
        "--tb=short"
    ])
    sys.exit(exit_code)


def run_server():
    """Run the FastAPI server (DEPRECATED)."""
    print("\n" + "=" * 60)
    print("⚠️  DEPRECATED: This server mode is no longer supported")
    print("=" * 60)
    print("\nThe authentication backend has been unified with the RAG backend.")
    print("\nTo start the unified server:")
    print("  cd ../backend")
    print("  uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload")
    print("\nEndpoints:")
    print("  RAG: https://e-book-physical-ai-humanoid-robotics.onrender.com/api/*")
    print("  Auth: https://e-book-physical-ai-humanoid-robotics.onrender.com/auth/*")
    print("  Docs: https://e-book-physical-ai-humanoid-robotics.onrender.com/docs")
    print("\n" + "=" * 60 + "\n")
    sys.exit(1)


def check_env():
    """Check if .env file exists and has required variables."""
    env_path = Path(".env")

    if not env_path.exists():
        print("⚠️  Warning: .env file not found!")
        print("\nCreating .env from .env.example...")

        example_path = Path(".env.example")
        if example_path.exists():
            import shutil
            shutil.copy(example_path, env_path)
            print("✅ Created .env file")
            print("\n⚠️  Please update the following variables in .env:")
            print("  - DATABASE_URL")
            print("  - SESSION_SECRET (run: python run.py --generate-secret)")
            print("\nThen run the server again.\n")
            sys.exit(1)
        else:
            print("❌ .env.example not found!")
            sys.exit(1)

    # Check for required variables
    with open(env_path) as f:
        content = f.read()

    if "change-this-to-a-secure-random-secret" in content:
        print("\n⚠️  Warning: SESSION_SECRET is still using default value!")
        print("\nGenerate a secure secret with:")
        print("  python run.py --generate-secret")
        print("\nThen update SESSION_SECRET in your .env file.\n")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Authentication Backend Quick Start"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run the test suite"
    )
    parser.add_argument(
        "--migrate",
        action="store_true",
        help="Run database migrations"
    )
    parser.add_argument(
        "--generate-secret",
        action="store_true",
        help="Generate a secure session secret"
    )

    args = parser.parse_args()

    if args.generate_secret:
        generate_secret()
    elif args.test:
        run_tests()
    elif args.migrate:
        asyncio.run(run_migrations())
    else:
        check_env()
        run_server()


if __name__ == "__main__":
    main()
