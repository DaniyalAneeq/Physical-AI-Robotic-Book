"""
⚠️  DEPRECATED - This config module is no longer used.

The authentication backend has been merged with the RAG backend.
All settings are now centralized in ../backend/app/config.py

For backwards compatibility, this module re-exports settings from the unified backend.
All auth routes should import settings from here to maintain compatibility.
"""

# Import settings from unified backend
import sys
from pathlib import Path

# Add parent directory to path to import from backend
parent_dir = Path(__file__).resolve().parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

# Import unified backend settings
try:
    from backend.app.config import get_settings
    settings = get_settings()
except ImportError:
    # Fallback for testing or standalone use
    import warnings
    warnings.warn(
        "Could not import unified backend settings. "
        "Using deprecated standalone auth settings. "
        "This should only happen in tests or standalone auth server."
    )

    from pydantic_settings import BaseSettings, SettingsConfigDict

    # Get the absolute path to the auth_backend directory
    AUTH_BACKEND_DIR = Path(__file__).parent.resolve()
    ENV_FILE_PATH = AUTH_BACKEND_DIR / ".env"

    class AuthSettings(BaseSettings):
        """
        Authentication settings loaded from environment variables.

        DEPRECATED: Use backend.app.config.Settings instead.
        """

        model_config = SettingsConfigDict(
            env_file=str(ENV_FILE_PATH),
            env_file_encoding="utf-8",
            case_sensitive=False,
            extra="ignore"
        )

        # Database
        database_url: str

        # Authentication
        session_secret: str = "change-this-to-a-secure-random-secret-in-production"
        session_cookie_name: str = "session_token"
        session_max_age_days: int = 30
        session_idle_timeout_days: int = 7

        # CORS
        cors_origins: str = "http://localhost:3000,http://localhost:8000,https://daniyalaneeq.github.io"

        # Google OAuth (optional)
        oauth_google_client_id: str = ""
        oauth_google_client_secret: str = ""
        oauth_google_redirect_uri: str = "http://localhost:8000/auth/oauth/google/callback"

        # GitHub OAuth (optional)
        oauth_github_client_id: str = ""
        oauth_github_client_secret: str = ""
        oauth_github_redirect_uri: str = "http://localhost:8000/auth/oauth/github/callback"

        # Frontend URL for OAuth redirects
        frontend_url: str = "http://localhost:3000/Physical-AI-Robotic-Book"

        # Cookie Configuration (environment-specific)
        secure_cookies: bool = False
        same_site_cookies: str = "lax"

        @property
        def cors_origins_list(self) -> list[str]:
            """Parse CORS origins from comma-separated string."""
            return [origin.strip() for origin in self.cors_origins.split(",")]

    # Fallback settings instance
    settings = AuthSettings()
