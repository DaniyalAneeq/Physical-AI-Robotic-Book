"""Environment configuration using Pydantic Settings."""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables.

    IMPORTANT: All sensitive values should be set via environment variables
    or a .env file. Never commit actual API keys to version control.
    """

    # OpenAI - REQUIRED: Set via OPENAI_API_KEY environment variable
    openai_api_key: str

    # Qdrant Cloud - REQUIRED: Set via environment variables
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "content-aidd-book"

    # Neon Postgres - REQUIRED: Set via NEON_DATABASE_URL environment variable
    neon_database_url: str

    # Server
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = False

    # Rate limiting
    rate_limit_requests: int = 60
    rate_limit_window_seconds: int = 60

    # Session
    session_expiry_hours: int = 24
    max_conversation_history: int = 20

    # Embedding
    embedding_model: str = "text-embedding-3-small"
    embedding_dimensions: int = 1536

    # Chat
    chat_model: str = "gpt-4o-mini"
    max_query_length: int = 1000
    max_chunks_retrieved: int = 5

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = "ignore"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
