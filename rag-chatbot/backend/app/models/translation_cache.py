"""Translation cache model for chatbot query translations."""

from datetime import datetime
from typing import Optional

from sqlalchemy import DateTime, Integer, String, Text
from sqlalchemy.orm import Mapped, mapped_column

from app.models.database import Base


class TranslationCache(Base):
    """
    Translation cache for chatbot query/response pairs.

    Stores translated query-response pairs to reduce OpenAI API calls
    and improve chatbot response latency.

    Attributes:
        id: Primary key (auto-increment)
        query_hash: SHA-256 hash of normalized query + language pair (unique)
        source_language: Source language code ('en' or 'ur')
        target_language: Target language code ('en' or 'ur')
        original_query: Verbatim user query
        translated_response: Cached translation result
        created_at: Cache entry creation timestamp
        last_accessed: Last cache hit timestamp
        access_count: Number of times this cached translation was used
    """

    __tablename__ = "translation_cache"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    query_hash: Mapped[str] = mapped_column(
        String(64), unique=True, nullable=False, index=True
    )
    source_language: Mapped[str] = mapped_column(String(5), nullable=False)
    target_language: Mapped[str] = mapped_column(String(5), nullable=False)
    original_query: Mapped[str] = mapped_column(Text, nullable=False)
    translated_response: Mapped[str] = mapped_column(Text, nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, default=datetime.utcnow
    )
    last_accessed: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, default=datetime.utcnow
    )
    access_count: Mapped[int] = mapped_column(Integer, nullable=False, default=1)

    def __repr__(self) -> str:
        return (
            f"<TranslationCache(id={self.id}, "
            f"{self.source_language}→{self.target_language}, "
            f"hits={self.access_count})>"
        )
