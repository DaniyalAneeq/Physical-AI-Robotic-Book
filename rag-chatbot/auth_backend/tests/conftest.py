"""Pytest configuration and fixtures for authentication tests."""

import os
import sys
from pathlib import Path

# Add backend directory to Python path
backend_dir = Path(__file__).resolve().parent.parent.parent / "backend"
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

# Set dummy environment variables before importing the app
os.environ["OPENAI_API_KEY"] = "test"
os.environ["QDRANT_URL"] = "http://localhost:6333"
os.environ["QDRANT_API_KEY"] = "test"
os.environ["DATABASE_URL"] = "sqlite+aiosqlite:///:memory:"

import pytest
import pytest_asyncio
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, async_sessionmaker
from httpx import AsyncClient

from backend.app.main import app
from auth_backend.database import get_db
from auth_backend.models.base import Base


# Test database URL (use in-memory SQLite for tests)
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"


# Configure pytest-asyncio
pytest_plugins = ('pytest_asyncio',)


@pytest_asyncio.fixture
async def engine():
    """Create async test database engine."""
    from auth_backend.models.user import User
    from auth_backend.models.session import Session
    from auth_backend.models.oauth_account import OAuthAccount

    engine = create_async_engine(TEST_DATABASE_URL, echo=False)
    async with engine.begin() as conn:
        # Create only the tables needed for auth testing (excluding onboarding_profiles which uses JSONB)
        await conn.run_sync(User.__table__.create, checkfirst=True)
        await conn.run_sync(Session.__table__.create, checkfirst=True)
        await conn.run_sync(OAuthAccount.__table__.create, checkfirst=True)
    yield engine
    await engine.dispose()


@pytest_asyncio.fixture
async def db_session(engine):
    """Create async database session for tests."""
    AsyncTestSession = async_sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )
    async with AsyncTestSession() as session:
        yield session
        # Rollback any pending transactions
        await session.rollback()
        await session.close()


@pytest_asyncio.fixture
async def client(db_session):
    """Create HTTP client for testing API endpoints."""
    from httpx import ASGITransport

    async def override_get_db():
        yield db_session

    app.dependency_overrides[get_db] = override_get_db

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as ac:
        yield ac

    app.dependency_overrides.clear()
