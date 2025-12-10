"""Embedding service using OpenAI text-embedding-3-small."""

from openai import AsyncOpenAI
from typing import Optional

from ..config import get_settings

settings = get_settings()


class EmbeddingService:
    """Service for generating text embeddings using OpenAI."""

    def __init__(self):
        """Initialize embedding service."""
        self._client: Optional[AsyncOpenAI] = None

    @property
    def client(self) -> AsyncOpenAI:
        """Lazy initialization of OpenAI client."""
        if self._client is None:
            self._client = AsyncOpenAI(api_key=settings.openai_api_key)
        return self._client

    async def validate_connection(self) -> bool:
        """Validate connection to OpenAI API."""
        try:
            # Simple test to check API key validity
            response = await self.client.embeddings.create(
                model=settings.embedding_model,
                input="test",
            )
            return len(response.data) > 0
        except Exception:
            return False

    async def embed(self, text: str) -> list[float]:
        """Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (1536 dimensions for text-embedding-3-small)
        """
        response = await self.client.embeddings.create(
            model=settings.embedding_model,
            input=text,
        )
        return response.data[0].embedding

    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        """Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        response = await self.client.embeddings.create(
            model=settings.embedding_model,
            input=texts,
        )

        # Sort by index to maintain order
        sorted_data = sorted(response.data, key=lambda x: x.index)
        return [item.embedding for item in sorted_data]


# Singleton instance
embedding_service = EmbeddingService()
