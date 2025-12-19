"""Session management service."""

import uuid
from datetime import datetime, timedelta
from typing import Optional, Tuple

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.models.session import Session
from auth_backend.models.user import User
from auth_backend.utils.security import generate_and_hash_token, hash_token


def get_cookie_attributes(secure: bool, same_site: str) -> dict:
    """
    Get cookie attributes based on environment configuration.

    This helper ensures cookies are configured correctly for the environment:
    - Development (HTTP): SameSite=Lax, Secure=False
    - Production (HTTPS): SameSite=None, Secure=True

    Args:
        secure: Whether to set the Secure flag (requires HTTPS)
        same_site: SameSite attribute value ("lax", "strict", or "none")

    Returns:
        dict: Cookie attributes ready for use with response.set_cookie()

    Example:
        >>> from auth_backend.config import settings
        >>> attrs = get_cookie_attributes(settings.secure_cookies, settings.same_site_cookies)
        >>> response.set_cookie(key="session", value=token, **attrs)
    """
    return {
        "httponly": True,
        "secure": secure,
        "samesite": same_site,
        "path": "/",
        "domain": None,  # Let browser set domain automatically for better compatibility
    }


class SessionService:
    """
    Service for managing user sessions.

    Handles session creation, validation, revocation, and cleanup.
    Follows Better Auth session management patterns.
    """

    def __init__(self, secret_key: str):
        """
        Initialize session service.

        Args:
            secret_key: Secret key for HMAC token hashing
        """
        self.secret_key = secret_key

    async def create_session(
        self,
        db: AsyncSession,
        user_id: uuid.UUID,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None,
    ) -> Tuple[Session, str]:
        """
        Create a new session for a user.

        Args:
            db: Database session
            user_id: User ID to create session for
            ip_address: IP address of the request (optional)
            user_agent: User agent string (optional)

        Returns:
            Tuple[Session, str]: (session_object, plain_token)
                The plain_token should be sent to client as HttpOnly cookie.
                The session object contains the hashed token stored in database.

        Example:
            >>> session, token = await session_service.create_session(
            ...     db, user_id, "192.168.1.1", "Mozilla/5.0..."
            ... )
            >>> # Set cookie: response.set_cookie("session_token", token, httponly=True)
        """
        # Generate token and hash
        plain_token, hashed_token = generate_and_hash_token(self.secret_key)

        # Create session
        session = Session(
            user_id=user_id,
            token_hash=hashed_token,
            ip_address=ip_address,
            user_agent=user_agent,
            last_used_at=datetime.utcnow(),
            expires_at=datetime.utcnow() + timedelta(days=30),  # 30-day absolute expiration
        )

        db.add(session)
        await db.commit()
        await db.refresh(session)

        return session, plain_token

    async def validate_session(
        self, db: AsyncSession, plain_token: str
    ) -> Optional[Tuple[Session, User]]:
        """
        Validate a session token and return session + user if valid.

        Updates last_used_at for sliding expiration.

        Args:
            db: Database session
            plain_token: Plain session token from cookie

        Returns:
            Optional[Tuple[Session, User]]: (session, user) if valid, None otherwise

        Validation checks:
        - Token hash exists in database
        - Session not expired
        - Session not revoked
        - User exists

        Example:
            >>> result = await session_service.validate_session(db, token)
            >>> if result:
            ...     session, user = result
            ...     print(f"Authenticated as {user.email}")
            ... else:
            ...     print("Invalid session")
        """
        # Hash the plain token
        hashed_token = hash_token(plain_token, self.secret_key)

        # Query session with user
        stmt = (
            select(Session, User)
            .join(User, Session.user_id == User.id)
            .where(Session.token_hash == hashed_token)
        )
        result = await db.execute(stmt)
        row = result.first()

        if not row:
            return None

        session, user = row

        # Check if session is valid
        if not session.is_valid():
            return None

        # Update last_used_at for sliding expiration
        session.last_used_at = datetime.utcnow()
        await db.commit()

        return session, user

    async def refresh_session(
        self,
        db: AsyncSession,
        old_plain_token: str,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None,
    ) -> Optional[Tuple[Session, str]]:
        """
        Refresh a session by creating a new session and invalidating the old one.

        This is used for token rotation to enhance security and extend session lifetime.

        Args:
            db: Database session
            old_plain_token: Current plain session token from cookie
            ip_address: IP address of the request (optional)
            user_agent: User agent string (optional)

        Returns:
            Optional[Tuple[Session, str]]: (new_session, new_plain_token) if successful, None if old session invalid

        Example:
            >>> new_session, new_token = await session_service.refresh_session(
            ...     db, old_token, "192.168.1.1", "Mozilla/5.0..."
            ... )
            >>> # Set new cookie: response.set_cookie("session_token", new_token, httponly=True)
        """
        # Validate the old session
        result = await self.validate_session(db, old_plain_token)
        if not result:
            return None

        old_session, user = result

        # Revoke the old session
        old_session.revoked = True

        # Create a new session
        new_session, new_plain_token = await self.create_session(
            db, user.id, ip_address, user_agent
        )

        await db.commit()
        await db.refresh(new_session)

        return new_session, new_plain_token

    async def revoke_session(self, db: AsyncSession, session_id: uuid.UUID) -> bool:
        """
        Revoke a session (logout).

        Args:
            db: Database session
            session_id: Session ID to revoke

        Returns:
            bool: True if session was revoked, False if not found

        Example:
            >>> success = await session_service.revoke_session(db, session_id)
            >>> if success:
            ...     print("Session revoked successfully")
        """
        stmt = select(Session).where(Session.id == session_id)
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if not session:
            return False

        session.revoked = True
        await db.commit()
        return True

    async def revoke_all_user_sessions(self, db: AsyncSession, user_id: uuid.UUID) -> int:
        """
        Revoke all sessions for a user.

        Useful for "logout from all devices" functionality.

        Args:
            db: Database session
            user_id: User ID to revoke all sessions for

        Returns:
            int: Number of sessions revoked

        Example:
            >>> count = await session_service.revoke_all_user_sessions(db, user_id)
            >>> print(f"Revoked {count} sessions")
        """
        stmt = select(Session).where(Session.user_id == user_id, Session.revoked == False)
        result = await db.execute(stmt)
        sessions = result.scalars().all()

        count = 0
        for session in sessions:
            session.revoked = True
            count += 1

        await db.commit()
        return count

    async def cleanup_expired_sessions(self, db: AsyncSession) -> int:
        """
        Delete expired sessions from database.

        Should be run periodically (e.g., daily cron job).

        Args:
            db: Database session

        Returns:
            int: Number of sessions deleted

        Example:
            >>> count = await session_service.cleanup_expired_sessions(db)
            >>> print(f"Cleaned up {count} expired sessions")
        """
        now = datetime.utcnow()
        stmt = select(Session).where(Session.expires_at < now)
        result = await db.execute(stmt)
        sessions = result.scalars().all()

        count = 0
        for session in sessions:
            await db.delete(session)
            count += 1

        await db.commit()
        return count
