"""Unit tests for application configuration."""

import os
import pytest
from unittest.mock import patch

from backend.app.config import get_settings, Settings

# Fixture to manage environment variables
@pytest.fixture(autouse=True)
def manage_environment():
    """Clear relevant environment variables and cache before and after each test."""
    get_settings.cache_clear()
    env_vars = [
        "ENVIRONMENT",
        "SECURE_COOKIES",
        "SAME_SITE_COOKIES",
        "CORS_ORIGINS",
    ]
    original_values = {var: os.environ.get(var) for var in env_vars}
    
    # Clear variables for a clean slate
    for var in env_vars:
        if var in os.environ:
            del os.environ[var]
    
    yield
    
    # Restore original values
    for var, value in original_values.items():
        if value is not None:
            os.environ[var] = value
        elif var in os.environ:
            del os.environ[var]
    get_settings.cache_clear()

def test_development_cookie_settings_default():
    """Test default cookie settings for development environment."""
    settings = get_settings()
    assert settings.environment == "development"
    assert settings.secure_cookies is False
    assert settings.same_site_cookies == "lax"

@patch.dict(os.environ, {"ENVIRONMENT": "development"})
def test_development_cookie_settings_explicit():
    """Test explicit cookie settings for development environment."""
    settings = get_settings()
    assert settings.environment == "development"
    assert settings.secure_cookies is False
    assert settings.same_site_cookies == "lax"

@patch.dict(os.environ, {"ENVIRONMENT": "production"})
def test_production_cookie_settings_default():
    """Test default cookie settings for production environment."""
    settings = get_settings()
    assert settings.environment == "production"
    assert settings.secure_cookies is True
    assert settings.same_site_cookies == "none"

@patch.dict(os.environ, {
    "ENVIRONMENT": "production",
    "SECURE_COOKIES": "true",
    "SAME_SITE_COOKIES": "none"
})
def test_production_cookie_settings_explicit():
    """Test explicit cookie settings for production environment."""
    settings = get_settings()
    assert settings.environment == "production"
    assert settings.secure_cookies is True
    assert settings.same_site_cookies == "none"

@patch.dict(os.environ, {"CORS_ORIGINS": "http://localhost:3000,https://app.example.com"})
def test_cors_origins_parsing():
    """Test parsing of CORS_ORIGINS from a comma-separated string."""
    settings = get_settings()
    assert settings.cors_origins_list == ["http://localhost:3000", "https://app.example.com"]

@patch.dict(os.environ, {"CORS_ORIGINS": "  http://localhost:3000  , https://app.example.com "})
def test_cors_origins_parsing_with_whitespace():
    """Test parsing of CORS_ORIGINS with extra whitespace."""
    settings = get_settings()
    assert settings.cors_origins_list == ["http://localhost:3000", "https://app.example.com"]

def test_cors_origins_default():
    """Test default CORS origins when environment variable is not set."""
    settings = get_settings()
    assert "http://localhost:3000" in settings.cors_origins_list
    assert "http://localhost:8000" in settings.cors_origins_list
