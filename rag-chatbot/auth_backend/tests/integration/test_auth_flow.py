"""Integration tests for authentication flow (register, login, logout)."""

import pytest
from httpx import AsyncClient


@pytest.mark.asyncio
async def test_register_success(client: AsyncClient):
    """Test successful user registration."""
    response = await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    assert response.status_code == 201
    data = response.json()

    assert data["user"]["email"] == "alice@example.com"
    assert data["user"]["name"] == "Alice Smith"
    assert "id" in data["user"]
    assert "session" in data
    assert data["message"] == "Account created successfully. You are now logged in."

    # Check session cookie is set
    assert "session_token" in response.cookies


@pytest.mark.asyncio
async def test_register_duplicate_email(client: AsyncClient):
    """Test registration with existing email fails."""
    # Register first user
    await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    # Try to register again with same email
    response = await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Duplicate",
            "password": "DifferentPass456",
        },
    )

    assert response.status_code == 409
    assert "already registered" in response.json()["detail"].lower()


@pytest.mark.asyncio
async def test_register_weak_password(client: AsyncClient):
    """Test registration with weak password fails."""
    response = await client.post(
        "/api/auth/register",
        json={
            "email": "bob@example.com",
            "name": "Bob Jones",
            "password": "short",  # Less than 8 characters
        },
    )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_login_success(client: AsyncClient):
    """Test successful login."""
    # Register user first
    await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    # Login
    response = await client.post(
        "/api/auth/login",
        json={
            "email": "alice@example.com",
            "password": "SecurePass123",
        },
    )

    assert response.status_code == 200
    data = response.json()

    assert data["user"]["email"] == "alice@example.com"
    assert data["message"] == "Logged in successfully."
    assert "session_token" in response.cookies


@pytest.mark.asyncio
async def test_login_wrong_password(client: AsyncClient):
    """Test login with incorrect password fails."""
    # Register user
    await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    # Try to login with wrong password
    response = await client.post(
        "/api/auth/login",
        json={
            "email": "alice@example.com",
            "password": "WrongPassword",
        },
    )

    assert response.status_code == 401
    assert "invalid email or password" in response.json()["detail"].lower()


@pytest.mark.asyncio
async def test_login_nonexistent_user(client: AsyncClient):
    """Test login with non-existent email fails."""
    response = await client.post(
        "/api/auth/login",
        json={
            "email": "notfound@example.com",
            "password": "SomePassword123",
        },
    )

    assert response.status_code == 401
    assert "invalid email or password" in response.json()["detail"].lower()


@pytest.mark.asyncio
async def test_logout_success(client: AsyncClient):
    """Test successful logout."""
    # Register and get session cookie
    register_response = await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    session_cookie = register_response.cookies.get("session_token")

    # Logout
    response = await client.post(
        "/api/auth/logout",
        cookies={"session_token": session_cookie},
    )

    assert response.status_code == 200
    assert response.json()["message"] == "Logged out successfully."


@pytest.mark.asyncio
async def test_logout_without_session(client: AsyncClient):
    """Test logout without session fails."""
    response = await client.post("/api/auth/logout")

    assert response.status_code == 401


@pytest.mark.asyncio
async def test_get_session_authenticated(client: AsyncClient):
    """Test getting session info when authenticated."""
    # Register and get session cookie
    register_response = await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )

    session_cookie = register_response.cookies.get("session_token")

    # Get session
    response = await client.get(
        "/api/auth/session",
        cookies={"session_token": session_cookie},
    )

    assert response.status_code == 200
    data = response.json()
    assert data["user"]["email"] == "alice@example.com"
    assert data["message"] == "Session is valid."


@pytest.mark.asyncio
async def test_get_session_unauthenticated(client: AsyncClient):
    """Test getting session without authentication fails."""
    response = await client.get("/api/auth/session")

    assert response.status_code == 401
    assert "not authenticated" in response.json()["detail"].lower()


@pytest.mark.asyncio
async def test_full_auth_flow(client: AsyncClient):
    """Test complete authentication flow: register -> logout -> login -> logout."""
    # 1. Register
    register_response = await client.post(
        "/api/auth/register",
        json={
            "email": "alice@example.com",
            "name": "Alice Smith",
            "password": "SecurePass123",
        },
    )
    assert register_response.status_code == 201

    # 2. Logout
    session_cookie = register_response.cookies.get("session_token")
    logout_response = await client.post(
        "/api/auth/logout",
        cookies={"session_token": session_cookie},
    )
    assert logout_response.status_code == 200

    # 3. Login again
    login_response = await client.post(
        "/api/auth/login",
        json={
            "email": "alice@example.com",
            "password": "SecurePass123",
        },
    )
    assert login_response.status_code == 200

    # 4. Verify session works
    new_session_cookie = login_response.cookies.get("session_token")
    session_response = await client.get(
        "/api/auth/session",
        cookies={"session_token": new_session_cookie},
    )
    assert session_response.status_code == 200

    # 5. Logout again
    final_logout = await client.post(
        "/api/auth/logout",
        cookies={"session_token": new_session_cookie},
    )
    assert final_logout.status_code == 200
