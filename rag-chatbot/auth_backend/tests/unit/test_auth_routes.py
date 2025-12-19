"""Unit tests for authentication routes (User Story 1: Email/Password Authentication)."""

import pytest
from httpx import AsyncClient
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.models.user import User
from auth_backend.models.session import Session
from auth_backend.config import settings


class TestRegisterEndpoint:
    """Test /auth/register endpoint (T015)."""

    @pytest.mark.asyncio
    async def test_register_sets_session_cookie_with_correct_attributes(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test registration sets session cookie with correct attributes."""
        # Arrange
        user_data = {
            "email": "testuser@example.com",
            "name": "Test User",
            "password": "SecurePassword123"
        }

        # Act
        response = await client.post("/api/register", json=user_data)

        # Assert
        assert response.status_code == 201, f"Expected 201, got {response.status_code}: {response.text}"

        # Verify response structure
        data = response.json()
        assert "user" in data
        assert "session" in data
        assert "message" in data
        assert "onboarding_required" in data
        assert data["user"]["email"] == "testuser@example.com"
        assert data["onboarding_required"] is True  # New user needs onboarding

        # Verify cookie is set
        cookie_header = response.cookies.get(settings.session_cookie_name)
        assert cookie_header is not None, "Session cookie not set"

        # Verify cookie attributes (check response headers)
        set_cookie_headers = response.headers.get_list("set-cookie")
        assert len(set_cookie_headers) > 0, "No Set-Cookie headers found"

        session_cookie_header = None
        for header in set_cookie_headers:
            if settings.session_cookie_name in header:
                session_cookie_header = header
                break

        assert session_cookie_header is not None, f"Session cookie '{settings.session_cookie_name}' not found in Set-Cookie headers"

        # Verify cookie attributes in header
        assert "HttpOnly" in session_cookie_header, "Cookie missing HttpOnly attribute"
        assert "Path=/" in session_cookie_header, "Cookie missing Path=/ attribute"
        assert "Max-Age=" in session_cookie_header, "Cookie missing Max-Age attribute"

        # Environment-specific attributes
        if settings.secure_cookies:
            assert "Secure" in session_cookie_header, "Cookie missing Secure attribute in production"
            assert "samesite=none" in session_cookie_header.lower(), "Cookie should have SameSite=None in production"
        else:
            assert "samesite=lax" in session_cookie_header.lower(), "Cookie should have SameSite=Lax in development"

    @pytest.mark.asyncio
    async def test_register_creates_user_in_database(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test registration creates user in database."""
        user_data = {
            "email": "dbtest@example.com",
            "name": "DB Test User",
            "password": "SecurePassword123"
        }

        response = await client.post("/auth/register", json=user_data)
        assert response.status_code == 201

        # Verify user exists in database
        stmt = select(User).where(User.email == "dbtest@example.com")
        result = await db_session.execute(stmt)
        user = result.scalar_one_or_none()

        assert user is not None, "User not created in database"
        assert user.name == "DB Test User"
        assert user.password_hash is not None, "Password hash not set"
        assert user.onboarding_completed is False, "New user should not have completed onboarding"

    @pytest.mark.asyncio
    async def test_register_duplicate_email_returns_409(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test registering duplicate email returns 409 Conflict."""
        user_data = {
            "email": "duplicate@example.com",
            "name": "First User",
            "password": "SecurePassword123"
        }

        # First registration
        response1 = await client.post("/auth/register", json=user_data)
        assert response1.status_code == 201

        # Second registration with same email
        response2 = await client.post("/auth/register", json=user_data)
        assert response2.status_code == 409
        assert "already registered" in response2.json()["detail"].lower()


class TestLoginEndpoint:
    """Test /auth/login endpoint (T016)."""

    @pytest.mark.asyncio
    async def test_login_sets_session_cookie_with_correct_attributes(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test login sets session cookie with correct attributes."""
        # Arrange: Create a user first
        user_data = {
            "email": "logintest@example.com",
            "name": "Login Test User",
            "password": "SecurePassword123"
        }
        await client.post("/auth/register", json=user_data)

        # Act: Login
        login_data = {
            "email": "logintest@example.com",
            "password": "SecurePassword123"
        }
        response = await client.post("/api/login", json=login_data)

        # Assert
        assert response.status_code == 200, f"Expected 200, got {response.status_code}: {response.text}"

        # Verify response structure
        data = response.json()
        assert "user" in data
        assert "session" in data
        assert "message" in data
        assert "onboarding_required" in data
        assert data["user"]["email"] == "logintest@example.com"

        # Verify cookie is set
        cookie_header = response.cookies.get(settings.session_cookie_name)
        assert cookie_header is not None, "Session cookie not set"

        # Verify cookie attributes
        set_cookie_headers = response.headers.get_list("set-cookie")
        session_cookie_header = None
        for header in set_cookie_headers:
            if settings.session_cookie_name in header:
                session_cookie_header = header
                break

        assert session_cookie_header is not None
        assert "HttpOnly" in session_cookie_header
        assert "Path=/" in session_cookie_header

    @pytest.mark.asyncio
    async def test_login_invalid_credentials_returns_401(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test login with invalid credentials returns 401."""
        # Arrange: Create a user
        user_data = {
            "email": "authtest@example.com",
            "name": "Auth Test User",
            "password": "CorrectPassword123"
        }
        await client.post("/auth/register", json=user_data)

        # Act: Try to login with wrong password
        login_data = {
            "email": "authtest@example.com",
            "password": "WrongPassword123"
        }
        response = await client.post("/api/login", json=login_data)

        # Assert
        assert response.status_code == 401
        assert "invalid" in response.json()["detail"].lower()

    @pytest.mark.asyncio
    async def test_login_nonexistent_user_returns_401(self, client: AsyncClient):
        """Test login with nonexistent user returns 401."""
        login_data = {
            "email": "nonexistent@example.com",
            "password": "AnyPassword123"
        }
        response = await client.post("/api/login", json=login_data)

        assert response.status_code == 401


class TestLogoutEndpoint:
    """Test /auth/logout endpoint (T017)."""

    @pytest.mark.asyncio
    async def test_logout_clears_session_cookie(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test logout clears session cookie with matching attributes."""
        # Arrange: Register and login
        user_data = {
            "email": "logouttest@example.com",
            "name": "Logout Test User",
            "password": "SecurePassword123"
        }
        register_response = await client.post("/auth/register", json=user_data)
        assert register_response.status_code == 201

        # Act: Logout
        logout_response = await client.post("/api/logout")

        # Assert
        assert logout_response.status_code == 200
        assert "logged out" in logout_response.json()["message"].lower()

        # Verify cookie is cleared (Max-Age=0 or expires in past)
        set_cookie_headers = logout_response.headers.get_list("set-cookie")
        session_cookie_header = None
        for header in set_cookie_headers:
            if settings.session_cookie_name in header:
                session_cookie_header = header
                break

        # Cookie should be deleted (Max-Age=0 or empty value)
        if session_cookie_header:
            # Some implementations set Max-Age=0, others set empty value
            assert (
                "Max-Age=0" in session_cookie_header or
                f"{settings.session_cookie_name}=;" in session_cookie_header
            ), "Cookie not properly cleared"

    @pytest.mark.asyncio
    async def test_logout_requires_authentication(self, client: AsyncClient):
        """Test logout without authentication returns 401."""
        from httpx import ASGITransport
        from auth_backend.main import app

        # Create a new client without session cookie
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://test") as new_client:
            response = await new_client.post("/api/logout")
            assert response.status_code == 401


class TestSessionEndpoint:
    """Test /auth/session endpoint (T018, T019)."""

    @pytest.mark.asyncio
    async def test_session_returns_401_when_no_cookie_present(
        self, client: AsyncClient
    ):
        """Test /auth/session returns 401 when no cookie present (T018)."""
        from httpx import ASGITransport
        from auth_backend.main import app

        # Create a new client without session cookie
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://test") as new_client:
            response = await new_client.get("/api/session")

            assert response.status_code == 401
            assert "detail" in response.json()

    @pytest.mark.asyncio
    async def test_session_returns_user_data_when_valid_cookie_present(
        self, client: AsyncClient, db_session: AsyncSession
    ):
        """Test /auth/session returns user data when valid cookie present (T019)."""
        # Arrange: Register a user (cookie will be set automatically)
        user_data = {
            "email": "sessiontest@example.com",
            "name": "Session Test User",
            "password": "SecurePassword123"
        }
        register_response = await client.post("/auth/register", json=user_data)
        assert register_response.status_code == 201

        # Act: Get session (cookie is automatically included by httpx client)
        response = await client.get("/api/session")

        # Assert
        assert response.status_code == 200, f"Expected 200, got {response.status_code}: {response.text}"

        data = response.json()
        assert "user" in data
        assert "session" in data
        assert "message" in data
        assert "onboarding_required" in data
        assert data["user"]["email"] == "sessiontest@example.com"
        assert data["user"]["name"] == "Session Test User"

    @pytest.mark.asyncio
    async def test_session_returns_401_for_invalid_cookie(
        self, client: AsyncClient
    ):
        """Test /auth/session returns 401 for invalid cookie."""
        from httpx import ASGITransport
        from auth_backend.main import app

        # Create a new client with invalid session cookie
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://test") as new_client:
            # Set invalid cookie
            new_client.cookies.set(settings.session_cookie_name, "invalid_token_value")

            response = await new_client.get("/api/session")

            assert response.status_code == 401


class TestSessionPersistence:
    """Test session persistence across requests."""

    @pytest.mark.asyncio
    async def test_session_persists_across_multiple_requests(
        self, client: AsyncClient
    ):
        """Test session cookie persists across multiple authenticated requests."""
        # Arrange: Register
        user_data = {
            "email": "persistence@example.com",
            "name": "Persistence Test",
            "password": "SecurePassword123"
        }
        register_response = await client.post("/auth/register", json=user_data)
        assert register_response.status_code == 201

        # Act & Assert: Make multiple requests
        for _ in range(3):
            response = await client.get("/api/session")
            assert response.status_code == 200
            assert response.json()["user"]["email"] == "persistence@example.com"

    @pytest.mark.asyncio
    async def test_logout_invalidates_future_requests(
        self, client: AsyncClient
    ):
        """Test that after logout, session is invalid for future requests."""
        # Arrange: Register
        user_data = {
            "email": "invalidate@example.com",
            "name": "Invalidate Test",
            "password": "SecurePassword123"
        }
        await client.post("/auth/register", json=user_data)

        # Verify session works
        response1 = await client.get("/auth/session")
        assert response1.status_code == 200

        # Act: Logout
        logout_response = await client.post("/api/logout")
        assert logout_response.status_code == 200

        # Assert: Session should be invalid after logout
        response2 = await client.get("/auth/session")
        assert response2.status_code == 401
