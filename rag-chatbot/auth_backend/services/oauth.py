"""OAuth service for Google OAuth 2.0 integration."""

import secrets
from typing import Optional, Tuple
from datetime import datetime

from authlib.integrations.starlette_client import OAuth
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.config import settings
from auth_backend.models.user import User
from auth_backend.models.oauth_account import OAuthAccount


class OAuthService:
    """
    Service for OAuth 2.0 authentication.

    Handles OAuth state generation/validation and account linking.
    Follows Better Auth OAuth patterns.
    """

    def __init__(self):
        """Initialize OAuth service (lazy loading)."""
        self._oauth = None

    @property
    def oauth(self) -> OAuth:
        """Lazy-load OAuth client to avoid blocking on import."""
        if self._oauth is None:
            self._oauth = OAuth()

            # Register Google OAuth provider
            if settings.oauth_google_client_id and settings.oauth_google_client_secret:
                self._oauth.register(
                    name="google",
                    client_id=settings.oauth_google_client_id,
                    client_secret=settings.oauth_google_client_secret,
                    server_metadata_url='https://accounts.google.com/.well-known/openid-configuration',
                    client_kwargs={
                        "scope": "openid email profile",
                        "prompt": "select_account"  # Always show account selection
                    },
                )

            # Register GitHub OAuth provider
            if settings.oauth_github_client_id and settings.oauth_github_client_secret:
                self._oauth.register(
                    name="github",
                    client_id=settings.oauth_github_client_id,
                    client_secret=settings.oauth_github_client_secret,
                    access_token_url="https://github.com/login/oauth/access_token",
                    authorize_url="https://github.com/login/oauth/authorize",
                    api_base_url="https://api.github.com/",
                    client_kwargs={"scope": "user:email"},
                )

        return self._oauth

    def generate_state(self) -> str:
        """
        Generate a cryptographically secure OAuth state token.

        Used for CSRF protection in OAuth flows.

        Returns:
            str: Random state token

        Example:
            >>> service = OAuthService()
            >>> state = service.generate_state()
            >>> len(state) >= 32
            True
        """
        return secrets.token_urlsafe(32)

    async def find_or_create_user_from_oauth(
        self, db: AsyncSession, provider: str, user_info: dict
    ) -> Tuple[User, OAuthAccount, bool]:
        """
        Find or create user from OAuth provider information.

        Implements account linking: if email exists, link OAuth to existing user.
        Otherwise, create new user.

        Args:
            db: Database session
            provider: OAuth provider name (e.g., "google")
            user_info: User info from OAuth provider containing:
                - sub: Provider user ID
                - email: User's email
                - name: User's name
                - picture: Profile picture URL (optional)

        Returns:
            Tuple[User, OAuthAccount, bool]: (user, oauth_account, is_new_user)

        Example:
            >>> user_info = {
            ...     "sub": "google_user_123",
            ...     "email": "alice@example.com",
            ...     "name": "Alice Smith"
            ... }
            >>> user, oauth_acc, is_new = await service.find_or_create_user_from_oauth(
            ...     db, "google", user_info
            ... )
        """
        provider_account_id = user_info.get("sub")
        email = user_info.get("email", "").lower()
        name = user_info.get("name", "")

        if not provider_account_id or not email:
            raise ValueError("OAuth user info missing required fields (sub, email)")

        # Check if OAuth account already exists
        stmt = select(OAuthAccount).where(
            OAuthAccount.provider == provider,
            OAuthAccount.provider_account_id == provider_account_id,
        )
        result = await db.execute(stmt)
        oauth_account = result.scalar_one_or_none()

        if oauth_account:
            # Existing OAuth account - load user
            stmt = select(User).where(User.id == oauth_account.user_id)
            result = await db.execute(stmt)
            user = result.scalar_one_or_none()
            return user, oauth_account, False

        # Check if user with this email exists (account linking)
        stmt = select(User).where(User.email == email)
        result = await db.execute(stmt)
        user = result.scalar_one_or_none()

        is_new_user = user is None

        if not user:
            # Create new user
            user = User(
                email=email,
                name=name,
                password_hash=None,  # OAuth users don't have password
            )
            db.add(user)
            await db.commit()
            await db.refresh(user)

        # Create OAuth account record
        oauth_account = OAuthAccount(
            user_id=user.id,
            provider=provider,
            provider_account_id=provider_account_id,
            access_token=None,  # Can store if needed
            refresh_token=None,  # Can store if needed
            expires_at=None,
            scope=None,
            token_type="Bearer",
        )
        db.add(oauth_account)
        await db.commit()
        await db.refresh(oauth_account)

        return user, oauth_account, is_new_user


# Singleton instance
oauth_service = OAuthService()
