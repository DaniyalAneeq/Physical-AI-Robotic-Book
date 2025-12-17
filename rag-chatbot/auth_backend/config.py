"""Configuration for authentication backend."""

from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict

# Get the absolute path to the auth_backend directory
AUTH_BACKEND_DIR = Path(__file__).parent.resolve()
ENV_FILE_PATH = AUTH_BACKEND_DIR / ".env"


class AuthSettings(BaseSettings):
    """
    Authentication settings loaded from environment variables.

    Follows Better Auth configuration patterns.
    """

    model_config = SettingsConfigDict(
        env_file=str(ENV_FILE_PATH), env_file_encoding="utf-8", case_sensitive=False, extra="ignore"
    )

    # Database
    database_url: str

    # Authentication
    session_secret: str = "change-this-to-a-secure-random-secret-in-production"
    session_cookie_name: str = "session_token"
    session_max_age_days: int = 30
    session_idle_timeout_days: int = 7

    # CORS
    cors_origins: str = "http://localhost:3000,http://localhost:8000"

    # Google OAuth (optional)
    oauth_google_client_id: str = ""
    oauth_google_client_secret: str = ""
    oauth_google_redirect_uri: str = "http://localhost:8000/auth/oauth/google/callback"

    # GitHub OAuth (optional)
    oauth_github_client_id: str = ""
    oauth_github_client_secret: str = ""
    oauth_github_redirect_uri: str = "http://localhost:8000/auth/oauth/github/callback"

    # Frontend URL for OAuth redirects
    frontend_url: str = "http://localhost:3000"

    # Security
    secure_cookies: bool = False  # Set to True in production (HTTPS)
    same_site_cookies: str = "lax"  # lax, strict, or none

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]


# Singleton settings instance
settings = AuthSettings()
