"""OAuth routes for Google and GitHub OAuth 2.0 authentication."""

from fastapi import APIRouter, Depends, HTTPException, Request, Response, status
from fastapi.responses import RedirectResponse
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.config import settings
from auth_backend.database import get_db
from auth_backend.schemas.auth import OAuthCallbackResponse
from auth_backend.schemas.session import SessionResponse
from auth_backend.schemas.user import UserResponse
from auth_backend.services.oauth import oauth_service
from auth_backend.services.session import SessionService

router = APIRouter(tags=["OAuth"])


@router.get("/google")
async def google_oauth_login(request: Request) -> RedirectResponse:
    """
    Initiate Google OAuth 2.0 login flow.

    Redirects user to Google's authorization page.

    **Query Parameters:**
    - None (state is generated automatically)

    **Response:**
    - 302 Redirect to Google OAuth authorization URL

    Example:
        ```
        GET /api/auth/oauth/google
        -> Redirects to: https://accounts.google.com/o/oauth2/auth?...
        ```
    """
    if not settings.oauth_google_client_id or not settings.oauth_google_client_secret:
        raise HTTPException(
            status_code=status.HTTP_501_NOT_IMPLEMENTED,
            detail="Google OAuth is not configured. Please set OAUTH_GOOGLE_CLIENT_ID and OAUTH_GOOGLE_CLIENT_SECRET.",
        )

    # Generate OAuth state for CSRF protection
    state = oauth_service.generate_state()

    # Store state in session (simplified - in production, use Redis or database)
    # For now, we'll pass it through the OAuth flow
    redirect_uri = settings.oauth_google_redirect_uri
    return await oauth_service.oauth.google.authorize_redirect(
        request, redirect_uri, state=state
    )


@router.get("/google/callback")
async def google_oauth_callback(
    request: Request,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> RedirectResponse:
    """
    Handle Google OAuth 2.0 callback.

    Exchanges authorization code for access token, retrieves user info,
    and creates/links user account.

    **Query Parameters:**
    - code: Authorization code from Google
    - state: CSRF state token (must match)

    **Response:**
    - Sets HttpOnly session cookie
    - Redirects to frontend application

    **Errors:**
    - 400 Bad Request: Invalid state or missing code
    - 500 Internal Server Error: OAuth exchange failed

    Example:
        ```
        GET /api/auth/oauth/google/callback?code=...&state=...
        -> Redirects to: http://localhost:3000/
        ```
    """
    try:
        # Exchange authorization code for access token
        token = await oauth_service.oauth.google.authorize_access_token(request)
    except Exception as e:
        # Redirect to frontend with error
        error_url = f"{settings.frontend_url}/login?error=oauth_failed"
        return RedirectResponse(url=error_url)

    # Get user info from Google
    user_info = token.get("userinfo")
    if not user_info:
        error_url = f"{settings.frontend_url}/login?error=no_user_info"
        return RedirectResponse(url=error_url)

    # Find or create user
    try:
        user, oauth_account, is_new_user = await oauth_service.find_or_create_user_from_oauth(
            db, "google", user_info
        )
    except Exception as e:
        error_url = f"{settings.frontend_url}/login?error=user_creation_failed"
        return RedirectResponse(url=error_url)

    # Create session
    session_service = SessionService(settings.session_secret)
    ip_address = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent")

    try:
        session, plain_token = await session_service.create_session(
            db, user.id, ip_address, user_agent
        )
    except Exception as e:
        error_url = f"{settings.frontend_url}/login?error=session_creation_failed"
        return RedirectResponse(url=error_url)

    # For cross-origin OAuth (localhost:8000 -> localhost:3000), we can't rely on cookies
    # being transferred in the redirect. Instead, pass the token in URL fragment.
    # Frontend will read it and call a special endpoint to establish the session.

    # Determine base redirect URL based on onboarding status
    if user.onboarding_completed:
        base_url = settings.frontend_url
        onboarding_param = "false"
    else:
        base_url = settings.frontend_url
        onboarding_param = "true"

    # Add token to URL fragment (not sent to server, client-side only for security)
    # Frontend auth callback page will read this and exchange it for a cookie
    import urllib.parse
    fragment_params = urllib.parse.urlencode({
        'session_token': plain_token,
        'onboarding_required': onboarding_param
    })
    redirect_url = f"{base_url}/auth/callback#{fragment_params}"

    return RedirectResponse(url=redirect_url)


@router.get("/github")
async def github_oauth_login(request: Request) -> RedirectResponse:
    """
    Initiate GitHub OAuth 2.0 login flow.

    Redirects user to GitHub's authorization page.

    **Query Parameters:**
    - None (state is generated automatically)

    **Response:**
    - 302 Redirect to GitHub OAuth authorization URL

    Example:
        ```
        GET /auth/oauth/github
        -> Redirects to: https://github.com/login/oauth/authorize?...
        ```
    """
    if not settings.oauth_github_client_id or not settings.oauth_github_client_secret:
        raise HTTPException(
            status_code=status.HTTP_501_NOT_IMPLEMENTED,
            detail="GitHub OAuth is not configured. Please set OAUTH_GITHUB_CLIENT_ID and OAUTH_GITHUB_CLIENT_SECRET.",
        )

    # Generate OAuth state for CSRF protection
    state = oauth_service.generate_state()

    redirect_uri = settings.oauth_github_redirect_uri
    return await oauth_service.oauth.github.authorize_redirect(
        request, redirect_uri, state=state
    )


@router.get("/github/callback")
async def github_oauth_callback(
    request: Request,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> RedirectResponse:
    """
    Handle GitHub OAuth 2.0 callback.

    Exchanges authorization code for access token, retrieves user info,
    and creates/links user account.

    **Query Parameters:**
    - code: Authorization code from GitHub
    - state: CSRF state token (must match)

    **Response:**
    - Sets HttpOnly session cookie
    - Redirects to frontend application

    **Errors:**
    - 400 Bad Request: Invalid state or missing code
    - 500 Internal Server Error: OAuth exchange failed

    Example:
        ```
        GET /auth/oauth/github/callback?code=...&state=...
        -> Redirects to: http://localhost:3000/
        ```
    """
    try:
        # Exchange authorization code for access token
        token = await oauth_service.oauth.github.authorize_access_token(request)
    except Exception as e:
        # Redirect to frontend with error
        error_url = f"{settings.frontend_url}/login?error=oauth_failed"
        return RedirectResponse(url=error_url)

    # Get user info from GitHub
    try:
        resp = await oauth_service.oauth.github.get("user", token=token)
        user_info = resp.json()

        # GitHub doesn't include email in the user endpoint by default
        # We need to fetch it separately
        if not user_info.get("email"):
            email_resp = await oauth_service.oauth.github.get("user/emails", token=token)
            emails = email_resp.json()
            # Get the primary verified email
            for email_data in emails:
                if email_data.get("primary") and email_data.get("verified"):
                    user_info["email"] = email_data["email"]
                    break
    except Exception as e:
        error_url = f"{settings.frontend_url}/login?error=no_user_info"
        return RedirectResponse(url=error_url)

    if not user_info.get("email"):
        error_url = f"{settings.frontend_url}/login?error=no_email"
        return RedirectResponse(url=error_url)

    # Find or create user
    try:
        user, oauth_account, is_new_user = await oauth_service.find_or_create_user_from_oauth(
            db, "github", user_info
        )
    except Exception as e:
        error_url = f"{settings.frontend_url}/login?error=user_creation_failed"
        return RedirectResponse(url=error_url)

    # Create session
    session_service = SessionService(settings.session_secret)
    ip_address = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent")

    try:
        session, plain_token = await session_service.create_session(
            db, user.id, ip_address, user_agent
        )
    except Exception as e:
        error_url = f"{settings.frontend_url}/login?error=session_creation_failed"
        return RedirectResponse(url=error_url)

    # For cross-origin OAuth (localhost:8000 -> localhost:3000), we can't rely on cookies
    # being transferred in the redirect. Instead, pass the token in URL fragment.
    # Frontend will read it and call a special endpoint to establish the session.

    # Determine base redirect URL based on onboarding status
    if user.onboarding_completed:
        base_url = settings.frontend_url
        onboarding_param = "false"
    else:
        base_url = settings.frontend_url
        onboarding_param = "true"

    # Add token to URL fragment (not sent to server, client-side only for security)
    # Frontend auth callback page will read this and exchange it for a cookie
    import urllib.parse
    fragment_params = urllib.parse.urlencode({
        'session_token': plain_token,
        'onboarding_required': onboarding_param
    })
    redirect_url = f"{base_url}/auth/callback#{fragment_params}"

    return RedirectResponse(url=redirect_url)
