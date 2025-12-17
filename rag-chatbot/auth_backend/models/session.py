"""Session model for authentication."""

import uuid
from datetime import datetime, timedelta
from typing import Optional

from sqlalchemy import Boolean, DateTime, ForeignKey, String, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from auth_backend.models.base import Base, TimestampMixin


class Session(Base, TimestampMixin):
    """
    Session model for user authentication.

    Stores active user sessions with cryptographic token hashing.
    Follows Better Auth session schema patterns.

    Attributes:
        id: Unique session identifier (UUID)
        user_id: Foreign key to users table
        token_hash: HMAC-SHA256 hashed session token
        last_used_at: Timestamp of last session use (for sliding expiration)
        expires_at: Absolute session expiration timestamp (30 days max)
        revoked: Whether session has been explicitly revoked
        ip_address: IP address of session creation (for audit)
        user_agent: User agent string (for multi-device tracking)
        created_at: Session creation timestamp
        updated_at: Last session update timestamp
    """

    __tablename__ = "auth_sessions"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False
    )
    token_hash: Mapped[str] = mapped_column(
        String(64), unique=True, nullable=False, index=True
    )
    last_used_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=datetime.utcnow, nullable=False
    )
    expires_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.utcnow() + timedelta(days=30),
        nullable=False,
    )
    revoked: Mapped[bool] = mapped_column(Boolean, default=False, nullable=False)
    ip_address: Mapped[Optional[str]] = mapped_column(String(45), nullable=True)
    user_agent: Mapped[Optional[str]] = mapped_column(Text, nullable=True)

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="sessions")

    def __repr__(self) -> str:
        return f"<Session(id={self.id}, user_id={self.user_id}, expires_at={self.expires_at})>"

    def is_valid(self, idle_timeout_days: int = 7) -> bool:
        """
        Check if session is valid (not expired, not idle, and not revoked).

        Validates:
        - Session is not explicitly revoked
        - Absolute expiration (30 days) not reached
        - Idle timeout (7 days since last_used_at) not reached

        Args:
            idle_timeout_days: Number of days before session expires due to inactivity

        Returns:
            bool: True if session is valid, False otherwise

        Example:
            >>> session.is_valid()  # Check with default 7-day idle timeout
            True
            >>> session.is_valid(idle_timeout_days=1)  # Check with 1-day idle timeout
            False
        """
        from datetime import timezone
        now = datetime.now(timezone.utc)

        # Check if revoked
        if self.revoked:
            return False

        # Check absolute expiration (30 days)
        if self.expires_at <= now:
            return False

        # Check idle timeout (7 days since last use)
        idle_cutoff = now - timedelta(days=idle_timeout_days)
        if self.last_used_at <= idle_cutoff:
            return False

        return True
