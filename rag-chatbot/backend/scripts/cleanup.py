#!/usr/bin/env python3
"""Cleanup script for expired conversations and query logs."""

import sys
from datetime import datetime, timedelta
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import delete

from app.config import get_settings
from app.models.conversation import Conversation
from app.models.database import get_db_context
from app.models.query_log import QueryLog

settings = get_settings()


def cleanup_old_conversations(days: int = 7) -> int:
    """
    Delete conversations older than specified days.

    Messages are cascade-deleted via foreign key constraint.

    Args:
        days: Number of days to retain conversations (default: 7)

    Returns:
        Number of conversations deleted
    """
    cutoff_date = datetime.utcnow() - timedelta(days=days)

    with get_db_context() as db:
        result = db.execute(
            delete(Conversation).where(Conversation.created_at < cutoff_date)
        )
        deleted_count = result.rowcount
        db.commit()

    print(f"Deleted {deleted_count} conversations older than {days} days")
    return deleted_count


def cleanup_old_query_logs(days: int = 30) -> int:
    """
    Delete query logs older than specified days.

    Args:
        days: Number of days to retain query logs (default: 30)

    Returns:
        Number of query logs deleted
    """
    cutoff_date = datetime.utcnow() - timedelta(days=days)

    with get_db_context() as db:
        result = db.execute(
            delete(QueryLog).where(QueryLog.created_at < cutoff_date)
        )
        deleted_count = result.rowcount
        db.commit()

    print(f"Deleted {deleted_count} query logs older than {days} days")
    return deleted_count


if __name__ == "__main__":
    print("Starting cleanup job...")
    print(f"Current time: {datetime.utcnow()}")

    # Cleanup conversations older than 7 days
    conv_count = cleanup_old_conversations(days=7)

    # Cleanup query logs older than 30 days
    log_count = cleanup_old_query_logs(days=30)

    print(f"\nCleanup complete:")
    print(f"  - {conv_count} conversations removed")
    print(f"  - {log_count} query logs removed")
