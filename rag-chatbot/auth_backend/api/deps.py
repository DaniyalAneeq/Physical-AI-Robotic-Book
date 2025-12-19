"""Authentication dependencies for FastAPI."""

import logging
from typing import Optional, Tuple

from fastapi import Cookie, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.config import settings
from auth_backend.database import get_db
from auth_backend.models.session import Session
from auth_backend.models.user import User
from auth_backend.services.session import SessionService

# Configure logger for authentication events
logger = logging.getLogger(__name__)


async def get_current_user(
    db: AsyncSession = Depends(get_db),
    session_token: Optional[str] = Cookie(None, alias=settings.session_cookie_name),
) -> User:
    """
    Dependency to get current authenticated user.

    Validates session token from HttpOnly cookie and returns user if valid.
    Raises 401 Unauthorized if session is invalid, expired, or missing.

    Args:
        db: Database session (injected)
        session_token: Session token from cookie (injected)

    Returns:
        User: Authenticated user object

    Raises:
        HTTPException: 401 Unauthorized if not authenticated

    Example:
        >>> @app.get("/api/user/profile")
        >>> async def get_profile(user: User = Depends(get_current_user)):
        ...     return {"email": user.email, "name": user.name}
    """
    if not session_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated. Please log in.",
        )

    session_service = SessionService(settings.session_secret)
    result = await session_service.validate_session(db, session_token)

    if not result:
        logger.warning(
            "Session validation failed: invalid or expired token",
            extra={"token_preview": session_token[:10] + "..." if len(session_token) > 10 else session_token}
        )
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session invalid or expired. Please log in again.",
        )

    session, user = result
    return user


async def get_current_user_optional(
    db: AsyncSession = Depends(get_db),
    session_token: Optional[str] = Cookie(None, alias=settings.session_cookie_name),
) -> Optional[User]:
    """
    Dependency to get current user if authenticated, None otherwise.

    Same as get_current_user but returns None instead of raising exception.
    Useful for endpoints that work for both authenticated and unauthenticated users.

    Args:
        db: Database session (injected)
        session_token: Session token from cookie (injected)

    Returns:
        Optional[User]: Authenticated user or None

    Example:
        >>> @app.get("/api/content")
        >>> async def get_content(user: Optional[User] = Depends(get_current_user_optional)):
        ...     if user:
        ...         return {"message": f"Hello {user.name}!"}
        ...     return {"message": "Hello guest!"}
    """
    if not session_token:
        return None

    session_service = SessionService(settings.session_secret)
    result = await session_service.validate_session(db, session_token)

    if not result:
        return None

    session, user = result
    return user


async def get_current_session_and_user(
    db: AsyncSession = Depends(get_db),
    session_token: Optional[str] = Cookie(None, alias=settings.session_cookie_name),
) -> Tuple[Session, User]:
    """
    Dependency to get current session and authenticated user.

    Similar to get_current_user but returns both session and user.
    Useful for operations that need to manipulate the session (e.g., logout).

    Args:
        db: Database session (injected)
        session_token: Session token from cookie (injected)

    Returns:
        Tuple[Session, User]: Current session and authenticated user

    Raises:
        HTTPException: 401 Unauthorized if not authenticated

    Example:
        >>> @app.post("/auth/logout")
        >>> async def logout(session_user: Tuple[Session, User] = Depends(get_current_session_and_user)):
        ...     session, user = session_user
        ...     # Can now revoke the session
    """
    if not session_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated. Please log in.",
        )

    session_service = SessionService(settings.session_secret)
    result = await session_service.validate_session(db, session_token)

    if not result:
        logger.warning(
            "Session validation failed: invalid or expired token",
            extra={"token_preview": session_token[:10] + "..." if len(session_token) > 10 else session_token}
        )
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session invalid or expired. Please log in again.",
        )

    return result  # Returns (session, user) tuple


async def get_current_user_with_onboarding(
    user: User = Depends(get_current_user),
) -> User:
    """
    Dependency to get current authenticated user who has completed onboarding.

    Validates that user is authenticated AND has completed onboarding flow.
    Raises 403 Forbidden if user hasn't completed onboarding.

    Args:
        user: Authenticated user (injected via get_current_user dependency)

    Returns:
        User: Authenticated user who has completed onboarding

    Raises:
        HTTPException: 403 Forbidden if onboarding not completed

    Example:
        >>> @app.post("/api/chat")
        >>> async def chat(user: User = Depends(get_current_user_with_onboarding)):
        ...     # User is authenticated AND onboarded
        ...     return {"message": "Chat available"}
    """
    if not user.onboarding_completed:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Onboarding required. Please complete your profile at /onboarding.",
            headers={"Location": "/onboarding"},  # Hint for frontend redirect
        )

    return user
