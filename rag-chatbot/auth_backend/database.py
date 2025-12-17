"""Database session management for authentication."""

from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

from auth_backend.config import settings


def get_async_database_url(url: str) -> str:
    """
    Convert database URL to asyncpg format.

    Removes incompatible query parameters like sslmode and adds asyncpg driver.
    """
    # Replace postgresql:// with postgresql+asyncpg://
    async_url = url.replace("postgresql://", "postgresql+asyncpg://")

    # Remove sslmode parameter if present (asyncpg handles SSL differently)
    if "sslmode=" in async_url:
        # Split URL and query string
        if "?" in async_url:
            base_url, query_string = async_url.split("?", 1)
            # Remove sslmode parameter
            params = [p for p in query_string.split("&") if not p.startswith("sslmode=")]
            if params:
                async_url = f"{base_url}?{'&'.join(params)}"
            else:
                async_url = base_url

    return async_url


# Create async engine
# Note: asyncpg handles SSL differently - it's configured via the connection URL
engine = create_async_engine(
    get_async_database_url(settings.database_url),
    echo=False,
    pool_pre_ping=True,
)

# Create session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
)


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency to get database session.

    Yields:
        AsyncSession: Database session

    Example:
        >>> @app.get("/")
        >>> async def root(db: AsyncSession = Depends(get_db)):
        ...     # Use db here
        ...     pass
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()
