"""Unit tests for authentication routes."""

import pytest
from httpx import AsyncClient
from unittest.mock import patch

# Mark all tests in this module as asyncio
pytestmark = pytest.mark.asyncio

async def test_register_sets_cookie(client: AsyncClient):
    """Test that registration sets a session cookie with correct attributes."""
    with patch("auth_backend.api.routes.auth.get_cookie_attributes") as mock_get_attributes:
        mock_get_attributes.return_value = {
            "httponly": True,
            "samesite": "lax",
            "secure": False,
            "path": "/",
        }
        
        response = await client.post(
            "/auth/register",
            json={"email": "test@example.com", "name": "Test User", "password": "password123"},
        )
        
        assert response.status_code == 201
        
        # Check that the Set-Cookie header is present
        assert "Set-Cookie" in response.headers
        
        # Parse the cookie
        cookie = response.headers["set-cookie"]
        assert "session_token=" in cookie
        assert "HttpOnly" in cookie
        assert "SameSite=Lax" in cookie
        assert "Path=/" in cookie

async def test_login_sets_cookie(client: AsyncClient):
    """Test that login sets a session cookie with correct attributes."""
    # First, register a user
    await client.post(
        "/auth/register",
        json={"email": "test@example.com", "name": "Test User", "password": "password123"},
    )
    
    with patch("auth_backend.api.routes.auth.get_cookie_attributes") as mock_get_attributes:
        mock_get_attributes.return_value = {
            "httponly": True,
            "samesite": "lax",
            "secure": False,
            "path": "/",
        }

        response = await client.post(
            "/auth/login",
            json={"email": "test@example.com", "password": "password123"},
        )
        
        assert response.status_code == 200
        
        # Check that the Set-Cookie header is present
        assert "Set-Cookie" in response.headers
        
        # Parse the cookie
        cookie = response.headers["set-cookie"]
        assert "session_token=" in cookie
        assert "HttpOnly" in cookie
        assert "SameSite=Lax" in cookie
        assert "Path=/" in cookie

async def test_logout_clears_cookie(client: AsyncClient):
    """Test that logout clears the session cookie."""
    # First, register and login a user
    register_response = await client.post(
        "/auth/register",
        json={"email": "test@example.com", "name": "Test User", "password": "password123"},
    )
    
    # Extract the cookie from the response
    cookie = register_response.headers["set-cookie"]
    
    with patch("auth_backend.api.routes.auth.get_cookie_attributes") as mock_get_attributes:
        mock_get_attributes.return_value = {
            "httponly": True,
            "samesite": "lax",
            "secure": False,
            "path": "/",
        }

        response = await client.post("/auth/logout", cookies={"session_token": cookie.split("=")[1].split(";")[0]})
        
        assert response.status_code == 200
        
        # Check that the Set-Cookie header is present and clears the cookie
        assert "Set-Cookie" in response.headers
        cleared_cookie = response.headers["set-cookie"]
        assert "session_token=;" in cleared_cookie or "session_token= " in cleared_cookie
        assert "Max-Age=0" in cleared_cookie

async def test_session_endpoint_no_cookie(client: AsyncClient):
    """Test that /auth/session returns 401 if no cookie is provided."""
    response = await client.get("/auth/session")
    assert response.status_code == 401

async def test_session_endpoint_with_cookie(client: AsyncClient):
    """Test that /auth/session returns user data if a valid cookie is provided."""
    # First, register a user
    register_response = await client.post(
        "/auth/register",
        json={"email": "test@example.com", "name": "Test User", "password": "password123"},
    )
    
    # Extract the cookie from the response
    cookie = register_response.headers["set-cookie"]
    
    response = await client.get("/auth/session", cookies={"session_token": cookie.split("=")[1].split(";")[0]})
    
    assert response.status_code == 200
    data = response.json()
    assert data["user"]["email"] == "test@example.com"
