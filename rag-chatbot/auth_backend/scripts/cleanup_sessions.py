"""
Session cleanup script for removing expired sessions.

This script should be run periodically (e.g., daily via cron) to clean up
expired sessions from the database.

Usage:
    python -m auth_backend.scripts.cleanup_sessions

Environment Variables:
    DATABASE_URL: PostgreSQL connection string (required)
    SESSION_SECRET: Session secret key (required)

Cron Example (daily at 2 AM):
    0 2 * * * cd /path/to/project && python -m auth_backend.scripts.cleanup_sessions >> /var/log/session_cleanup.log 2>&1
"""

import asyncio
import logging
import sys
from datetime import datetime

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

from auth_backend.config import settings
from auth_backend.services.session import SessionService

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def cleanup_expired_sessions():
    """
    Main cleanup function.

    Connects to database and removes all expired sessions.
    """
    logger.info("Starting session cleanup...")
    start_time = datetime.utcnow()

    try:
        # Create async database engine
        engine = create_async_engine(settings.database_url, echo=False)
        async_session_factory = sessionmaker(
            engine, class_=AsyncSession, expire_on_commit=False
        )

        async with async_session_factory() as db:
            session_service = SessionService(settings.session_secret)
            count = await session_service.cleanup_expired_sessions(db)

        await engine.dispose()

        elapsed = (datetime.utcnow() - start_time).total_seconds()
        logger.info(
            f"Session cleanup completed successfully. "
            f"Removed {count} expired sessions in {elapsed:.2f}s"
        )
        return count

    except Exception as e:
        logger.error(f"Session cleanup failed: {e}", exc_info=True)
        sys.exit(1)


def main():
    """Entry point for the cleanup script."""
    logger.info("=" * 60)
    logger.info("Session Cleanup Script")
    logger.info("=" * 60)

    # Run the cleanup
    count = asyncio.run(cleanup_expired_sessions())

    logger.info("=" * 60)
    logger.info(f"Cleanup complete. {count} sessions removed.")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
