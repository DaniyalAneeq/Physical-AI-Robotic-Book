"""Authentication handler for tests."""

import logging
import os
from typing import Any

import httpx
from playwright.async_api import BrowserContext, Page

logger = logging.getLogger(__name__)


class AuthenticationError(Exception):
    """Raised when authentication fails."""
    pass


class AuthHandler:
    """
    Handles authentication for browser and API tests.

    Supports:
    - Better Auth (browser and API)
    - OAuth2
    - JWT Bearer tokens
    - Session cookies
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize authentication handler.

        Args:
            config: Configuration dictionary with auth settings
        """
        self.config = config
        self.auth_config = config.get("auth", {})

    async def authenticate_browser(
        self, context: BrowserContext, method: str
    ) -> None:
        """
        Authenticate browser context using specified method.

        Args:
            context: Playwright browser context
            method: Authentication method (better_auth, oauth2, etc.)

        Raises:
            AuthenticationError: If authentication fails
        """
        if method == "none":
            return

        if method == "better_auth":
            await self._authenticate_better_auth_browser(context)
        elif method == "oauth2":
            await self._authenticate_oauth2_browser(context)
        else:
            raise AuthenticationError(f"Unsupported browser auth method: {method}")

    async def _authenticate_better_auth_browser(
        self, context: BrowserContext
    ) -> None:
        """
        Authenticate using Better Auth endpoints (browser).

        Args:
            context: Playwright browser context

        Raises:
            AuthenticationError: If authentication fails
        """
        try:
            email = os.getenv("TEST_AUTH_EMAIL")
            password = os.getenv("TEST_AUTH_PASSWORD")
            base_url = self.auth_config.get("base_url", "http://localhost:3000")

            if not email or not password:
                raise AuthenticationError(
                    "TEST_AUTH_EMAIL and TEST_AUTH_PASSWORD must be set"
                )

            page = await context.new_page()

            # Navigate to Better Auth login endpoint
            login_url = f"{base_url}/api/auth/signin"
            logger.info(f"Authenticating via Better Auth: {login_url}")

            await page.goto(login_url)

            # Fill credentials
            await page.fill('[name="email"]', email)
            await page.fill('[name="password"]', password)
            await page.click('button[type="submit"]')

            # Wait for redirect (Better Auth handles session creation)
            await page.wait_for_url(f"{base_url}/dashboard", timeout=10000)

            logger.info("Browser authentication successful")

            # Close the page but keep the context (session persists)
            await page.close()

        except Exception as e:
            raise AuthenticationError(f"Better Auth browser login failed: {e}")

    async def _authenticate_oauth2_browser(self, context: BrowserContext) -> None:
        """
        Authenticate using OAuth2 (browser).

        Args:
            context: Playwright browser context

        Raises:
            AuthenticationError: If authentication fails
        """
        # Placeholder for OAuth2 implementation
        raise AuthenticationError("OAuth2 browser authentication not yet implemented")

    async def authenticate_api(
        self, client: httpx.AsyncClient, method: str
    ) -> None:
        """
        Authenticate API client using specified method.

        Args:
            client: httpx AsyncClient
            method: Authentication method (jwt_bearer, session_cookie, etc.)

        Raises:
            AuthenticationError: If authentication fails
        """
        if method == "none":
            return

        if method in ["better_auth", "jwt_bearer"]:
            await self._authenticate_jwt_bearer(client)
        elif method == "session_cookie":
            await self._authenticate_session_cookie(client)
        else:
            raise AuthenticationError(f"Unsupported API auth method: {method}")

    async def _authenticate_jwt_bearer(self, client: httpx.AsyncClient) -> None:
        """
        Authenticate using JWT bearer token.

        Args:
            client: httpx AsyncClient

        Raises:
            AuthenticationError: If authentication fails
        """
        try:
            email = os.getenv("TEST_AUTH_EMAIL")
            password = os.getenv("TEST_AUTH_PASSWORD")
            base_url = self.config.get("backend_url", "http://localhost:8000")

            if not email or not password:
                raise AuthenticationError(
                    "TEST_AUTH_EMAIL and TEST_AUTH_PASSWORD must be set"
                )

            # Login via Better Auth endpoint
            login_url = f"{base_url}/api/auth/signin"
            logger.info(f"Authenticating API via JWT: {login_url}")

            response = await client.post(
                login_url,
                json={"email": email, "password": password},
            )

            if response.status_code not in [200, 201]:
                raise AuthenticationError(
                    f"Login failed with status {response.status_code}: {response.text}"
                )

            # Extract token from response
            response_data = response.json()
            token = response_data.get("access_token")

            if not token:
                raise AuthenticationError("No access_token in response")

            # Add to client headers for subsequent requests
            client.headers["Authorization"] = f"Bearer {token}"

            logger.info("API authentication successful")

        except httpx.HTTPError as e:
            raise AuthenticationError(f"JWT authentication failed: {e}")

    async def _authenticate_session_cookie(self, client: httpx.AsyncClient) -> None:
        """
        Authenticate using session cookie.

        Args:
            client: httpx AsyncClient

        Raises:
            AuthenticationError: If authentication fails
        """
        try:
            email = os.getenv("TEST_AUTH_EMAIL")
            password = os.getenv("TEST_AUTH_PASSWORD")
            base_url = self.config.get("backend_url", "http://localhost:8000")

            if not email or not password:
                raise AuthenticationError(
                    "TEST_AUTH_EMAIL and TEST_AUTH_PASSWORD must be set"
                )

            # Login to get session cookie
            login_url = f"{base_url}/api/auth/signin"
            logger.info(f"Authenticating API via session cookie: {login_url}")

            response = await client.post(
                login_url,
                json={"email": email, "password": password},
            )

            if response.status_code not in [200, 201]:
                raise AuthenticationError(
                    f"Login failed with status {response.status_code}: {response.text}"
                )

            # Session cookie is automatically stored in client.cookies
            logger.info("API session authentication successful")

        except httpx.HTTPError as e:
            raise AuthenticationError(f"Session cookie authentication failed: {e}")
