"""Authentication routes: register, login, logout."""

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request, Response, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.config import settings
from auth_backend.database import get_db
from auth_backend.models.user import User
from auth_backend.schemas.auth import AuthResponse, LoginRequest, RegisterRequest
from auth_backend.schemas.session import SessionResponse
from auth_backend.schemas.user import UserResponse
from auth_backend.services.password import password_service
from auth_backend.services.session import SessionService
from auth_backend.api.deps import get_current_user, get_current_session_and_user

router = APIRouter(tags=["Authentication"])


@router.post("/register", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def register(
    request_data: RegisterRequest,
    request: Request,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> AuthResponse:
    """
    Register a new user with email and password.

    Creates a new user account and establishes a session.

    **Request Body:**
    - email: Valid email address
    - name: Display name (1-255 characters)
    - password: Password (minimum 8 characters)

    **Response:**
    - Sets HttpOnly session cookie
    - Returns user and session information

    **Errors:**
    - 409 Conflict: Email already registered
    - 400 Bad Request: Invalid input (handled by Pydantic)

    Example:
        ```json
        POST /api/auth/register
        {
          "email": "alice@example.com",
          "name": "Alice Smith",
          "password": "SecurePass123"
        }
        ```
    """
    # Check if email already exists
    stmt = select(User).where(User.email == request_data.email.lower())
    result = await db.execute(stmt)
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered. Please log in instead.",
        )

    # Hash password
    password_hash = password_service.hash_password(request_data.password)

    # Create user
    user = User(
        email=request_data.email.lower(),
        name=request_data.name,
        password_hash=password_hash,
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)

    # Create session
    session_service = SessionService(settings.session_secret)
    ip_address = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent")

    session, plain_token = await session_service.create_session(
        db, user.id, ip_address, user_agent
    )

    # Set session cookie
    response.set_cookie(
        key=settings.session_cookie_name,
        value=plain_token,
        httponly=True,
        secure=settings.secure_cookies,
        samesite=settings.same_site_cookies,
        max_age=settings.session_max_age_days * 24 * 60 * 60,  # Convert days to seconds
    )

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse.model_validate(session),
        message="Account created successfully. You are now logged in.",
        onboarding_required=not user.onboarding_completed,
    )


@router.post("/login", response_model=AuthResponse)
async def login(
    request_data: LoginRequest,
    request: Request,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> AuthResponse:
    """
    Log in with email and password.

    Authenticates user and establishes a session.

    **Request Body:**
    - email: User's email address
    - password: User's password

    **Response:**
    - Sets HttpOnly session cookie
    - Returns user and session information

    **Errors:**
    - 401 Unauthorized: Invalid email or password
      (Generic error to prevent email enumeration)

    Example:
        ```json
        POST /api/auth/login
        {
          "email": "alice@example.com",
          "password": "SecurePass123"
        }
        ```
    """
    # Find user by email (case-insensitive)
    stmt = select(User).where(User.email == request_data.email.lower())
    result = await db.execute(stmt)
    user = result.scalar_one_or_none()

    # Check if user exists and password is correct
    if not user or not user.password_hash:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password.",
        )

    if not password_service.verify_password(request_data.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password.",
        )

    # Create session
    session_service = SessionService(settings.session_secret)
    ip_address = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent")

    session, plain_token = await session_service.create_session(
        db, user.id, ip_address, user_agent
    )

    # Set session cookie
    response.set_cookie(
        key=settings.session_cookie_name,
        value=plain_token,
        httponly=True,
        secure=settings.secure_cookies,
        samesite=settings.same_site_cookies,
        max_age=settings.session_max_age_days * 24 * 60 * 60,
    )

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse.model_validate(session),
        message="Logged in successfully.",
        onboarding_required=not user.onboarding_completed,
    )


@router.post("/logout")
async def logout(
    response: Response,
    session_user: tuple = Depends(get_current_session_and_user),
    db: AsyncSession = Depends(get_db),
) -> dict:
    """
    Log out current user.

    Revokes the current session from database and clears the session cookie.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - Revokes session in database
    - Clears session cookie
    - Returns success message

    Example:
        ```
        POST /auth/logout
        Cookie: session_token=abc123xyz
        ```
    """
    session, user = session_user

    # Revoke session in database (proper cleanup)
    session_service = SessionService(settings.session_secret)
    await session_service.revoke_session(db, session.id)

    # Clear session cookie
    response.delete_cookie(
        key=settings.session_cookie_name,
        httponly=True,
        secure=settings.secure_cookies,
        samesite=settings.same_site_cookies,
    )

    return {"message": "Logged out successfully."}


@router.get("/me", response_model=UserResponse)
async def get_current_user_info(
    user: User = Depends(get_current_user),
) -> UserResponse:
    """
    Get current authenticated user information.

    Returns user details for the authenticated user.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - User information

    **Errors:**
    - 401 Unauthorized: Not authenticated or session invalid

    Example:
        ```
        GET /auth/me
        Cookie: session_token=abc123xyz
        -> {
            "id": "...",
            "email": "alice@example.com",
            "name": "Alice Smith",
            "onboarding_completed": false,
            ...
        }
        ```
    """
    return UserResponse.model_validate(user)


@router.post("/refresh", response_model=AuthResponse)
async def refresh_session(
    request: Request,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> AuthResponse:
    """
    Refresh the current session.

    Creates a new session token and invalidates the old one.
    Useful for extending session lifetime or rotating tokens.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - Sets new HttpOnly session cookie
    - Returns user and new session information

    **Errors:**
    - 401 Unauthorized: Not authenticated or session invalid

    Example:
        ```
        POST /auth/refresh
        Cookie: session_token=old_token
        -> Sets new cookie and returns session info
        ```
    """
    # Get old token from cookie
    old_token = request.cookies.get(settings.session_cookie_name)
    if not old_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="No session token provided.",
        )

    # Refresh session (creates new, revokes old)
    session_service = SessionService(settings.session_secret)
    ip_address = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent")

    result = await session_service.refresh_session(
        db, old_token, ip_address, user_agent
    )

    if not result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session token.",
        )

    session, plain_token = result

    # Get user for response
    stmt = select(User).where(User.id == session.user_id)
    result = await db.execute(stmt)
    user = result.scalar_one()

    # Set new session cookie
    response.set_cookie(
        key=settings.session_cookie_name,
        value=plain_token,
        httponly=True,
        secure=settings.secure_cookies,
        samesite=settings.same_site_cookies,
        max_age=settings.session_max_age_days * 24 * 60 * 60,
    )

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse.model_validate(session),
        message="Session refreshed successfully.",
        onboarding_required=not user.onboarding_completed,
    )


@router.get("/session", response_model=AuthResponse)
async def get_session(
    session_user: tuple = Depends(get_current_session_and_user),
) -> AuthResponse:
    """
    Get current session information.

    Returns the current authenticated user and session details.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - User and session information

    **Errors:**
    - 401 Unauthorized: Not authenticated or session invalid

    Example:
        ```
        GET /auth/session
        Cookie: session_token=abc123xyz
        -> {
            "user": {...},
            "session": {...},
            "message": "Session retrieved successfully.",
            "onboarding_required": false
        }
        ```
    """
    session, user = session_user

    return AuthResponse(
        user=UserResponse.model_validate(user),
        session=SessionResponse.model_validate(session),
        message="Session retrieved successfully.",
        onboarding_required=not user.onboarding_completed,
    )
