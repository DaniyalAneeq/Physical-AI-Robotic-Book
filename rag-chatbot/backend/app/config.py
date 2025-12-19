"""Application configuration loaded from environment variables."""

from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # OpenAI Configuration
    openai_api_key: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "content-aidd-book"

    # Database Configuration
    database_url: str

    # Application Configuration
    environment: str = "development"
    log_level: str = "INFO"
    cors_origins: str = "http://localhost:3000,http://localhost:8000,https://ibrahimkhalildev.github.io"

    # Embedding Configuration
    embedding_model: str = "text-embedding-3-small"
    embedding_dimensions: int = 1536

    # Content Paths
    textbook_docs_path: str = "../../AIdd-book/docs"

    # Authentication Configuration
    session_secret: str = "change-me-in-production"
    session_cookie_name: str = "session_token"
    session_max_age_days: int = 30
    session_idle_timeout_days: int = 7

    # Cookie Configuration (environment-specific)
    @property
    def secure_cookies(self) -> bool:
        """Use secure cookies in production."""
        return self.is_production

    @property
    def same_site_cookies(self) -> str:
        """Use 'none' for SameSite in production, 'lax' otherwise."""
        return "none" if self.is_production else "lax"

    # OAuth - Google
    oauth_google_client_id: str = ""
    oauth_google_client_secret: str = ""
    oauth_google_redirect_uri: str = "http://localhost:8000/auth/oauth/google/callback"

    # OAuth - GitHub
    oauth_github_client_id: str = ""
    oauth_github_client_secret: str = ""
    oauth_github_redirect_uri: str = "http://localhost:8000/auth/oauth/github/callback"

    # Frontend URL (include base path for Docusaurus)
    frontend_url: str = "http://localhost:3000/Physical-AI-Robotic-Book"

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment.lower() == "production"


@lru_cache
def get_settings() -> Settings:
    """Get cached application settings."""
    return Settings()
