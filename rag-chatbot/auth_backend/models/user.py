"""User model for authentication."""

import uuid
from typing import Optional

from sqlalchemy import Boolean, String, Text
from sqlalchemy.dialects.postgresql import CITEXT, UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship
from sqlalchemy import DateTime

from auth_backend.models.base import Base, TimestampMixin


class User(Base, TimestampMixin):
    """
    User account model.

    Stores user information for both email/password and OAuth authentication.
    Follows Better Auth user schema patterns.

    Attributes:
        id: Unique user identifier (UUID)
        email: User's email address (case-insensitive, unique)
        name: User's display name
        password_hash: Argon2id hashed password (null for OAuth-only users)
        email_verified_at: Timestamp when email was verified (nullable)
        onboarding_completed: Flag indicating if user completed onboarding flow
        created_at: Account creation timestamp
        updated_at: Last account update timestamp
    """

    __tablename__ = "users"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    email: Mapped[str] = mapped_column(
        String(255), unique=True, nullable=False, index=True
    )
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    password_hash: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    email_verified_at: Mapped[Optional[DateTime]] = mapped_column(
        DateTime(timezone=True), nullable=True
    )
    onboarding_completed: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="false", index=True
    )
    preferred_locale: Mapped[str] = mapped_column(
        String(5), nullable=False, default="en", server_default="en", index=True
    )

    # Relationships
    sessions: Mapped[list["Session"]] = relationship(
        "Session", back_populates="user", cascade="all, delete-orphan"
    )
    oauth_accounts: Mapped[list["OAuthAccount"]] = relationship(
        "OAuthAccount", back_populates="user", cascade="all, delete-orphan"
    )
    onboarding_profile: Mapped[Optional["OnboardingProfile"]] = relationship(
        "OnboardingProfile", back_populates="user", uselist=False
    )

    def __repr__(self) -> str:
        return f"<User(id={self.id}, email={self.email}, name={self.name})>"
