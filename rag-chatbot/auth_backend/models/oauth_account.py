"""OAuth account model for third-party authentication."""

import uuid
from datetime import datetime
from typing import Optional

from sqlalchemy import DateTime, ForeignKey, String, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from auth_backend.models.base import Base, TimestampMixin


class OAuthAccount(Base, TimestampMixin):
    """
    OAuth account model for third-party authentication providers.

    Links user accounts to OAuth providers (Google, GitHub, etc.).
    Follows Better Auth oauth_accounts schema patterns.

    Attributes:
        id: Unique OAuth account identifier (UUID)
        user_id: Foreign key to users table
        provider: OAuth provider name (e.g., 'google', 'github')
        provider_account_id: User's unique ID from the OAuth provider
        access_token: OAuth access token (encrypted recommended)
        refresh_token: OAuth refresh token (encrypted recommended)
        expires_at: Access token expiration timestamp
        scope: OAuth scopes granted
        token_type: Token type (usually 'Bearer')
        created_at: OAuth account creation timestamp
        updated_at: Last OAuth account update timestamp
    """

    __tablename__ = "oauth_accounts"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False
    )
    provider: Mapped[str] = mapped_column(String(50), nullable=False)
    provider_account_id: Mapped[str] = mapped_column(String(255), nullable=False)
    access_token: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    refresh_token: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    expires_at: Mapped[Optional[datetime]] = mapped_column(
        DateTime(timezone=True), nullable=True
    )
    scope: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    token_type: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="oauth_accounts")

    def __repr__(self) -> str:
        return f"<OAuthAccount(id={self.id}, provider={self.provider}, user_id={self.user_id})>"

    class Config:
        """Pydantic model config."""

        from_attributes = True
