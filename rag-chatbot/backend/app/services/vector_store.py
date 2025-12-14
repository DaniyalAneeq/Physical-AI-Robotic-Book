"""Qdrant vector store service with circuit breaker."""

import logging
import time
from functools import lru_cache
from typing import Any

from qdrant_client import QdrantClient
from qdrant_client.http import models
from tenacity import retry, stop_after_attempt, wait_exponential

from app.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class CircuitBreaker:
    """Simple circuit breaker implementation."""

    def __init__(self, failure_threshold: int = 5, reset_timeout: int = 60):
        self.failures = 0
        self.threshold = failure_threshold
        self.last_failure: float | None = None
        self.reset_timeout = reset_timeout

    def is_open(self) -> bool:
        """Check if circuit breaker is open (blocking requests)."""
        if self.failures >= self.threshold:
            if self.last_failure and time.time() - self.last_failure > self.reset_timeout:
                self.failures = 0
                return False
            return True
        return False

    def record_failure(self) -> None:
        """Record a failure."""
        self.failures += 1
        self.last_failure = time.time()

    def record_success(self) -> None:
        """Record a success (reset failure count)."""
        self.failures = 0


class VectorStore:
    """Qdrant vector store wrapper with circuit breaker and retry logic."""

    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name
        self.circuit_breaker = CircuitBreaker()

    async def health_check(self) -> bool:
        """Check if Qdrant is healthy."""
        if self.circuit_breaker.is_open():
            raise ConnectionError("Circuit breaker is open")
        try:
            self.client.get_collections()
            self.circuit_breaker.record_success()
            return True
        except Exception as e:
            self.circuit_breaker.record_failure()
            raise e

    def collection_exists(self) -> bool:
        """Check if the collection exists."""
        try:
            collections = self.client.get_collections()
            return any(c.name == self.collection_name for c in collections.collections)
        except Exception as e:
            logger.error(f"Error checking collection existence: {e}")
            return False

    def create_collection(self) -> None:
        """Create the vector collection with proper configuration."""
        if self.collection_exists():
            logger.info(f"Collection {self.collection_name} already exists")
            return

        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=settings.embedding_dimensions,
                distance=models.Distance.COSINE,
            ),
            optimizers_config=models.OptimizersConfigDiff(
                default_segment_number=2,
            ),
        )
        logger.info(f"Created collection: {self.collection_name}")

        # Create payload indexes for filtering
        self.client.create_payload_index(
            collection_name=self.collection_name,
            field_name="module",
            field_schema=models.PayloadSchemaType.KEYWORD,
        )
        self.client.create_payload_index(
            collection_name=self.collection_name,
            field_name="chapter",
            field_schema=models.PayloadSchemaType.KEYWORD,
        )
        logger.info("Created payload indexes for module and chapter")

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
    def upsert_points(self, points: list[models.PointStruct]) -> None:
        """Upsert points to the collection with retry."""
        if self.circuit_breaker.is_open():
            raise ConnectionError("Circuit breaker is open")
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
            self.circuit_breaker.record_success()
        except Exception as e:
            self.circuit_breaker.record_failure()
            raise e

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
    def search(
        self,
        query_vector: list[float],
        limit: int = 5,
        module_filter: str | None = None,
    ) -> list[Any]:
        """Search for similar vectors with optional module filter.

        Returns:
            List of ScoredPoint objects from Qdrant with .score and .payload attributes
        """
        if self.circuit_breaker.is_open():
            raise ConnectionError("Circuit breaker is open")

        try:
            # Build filter if module specified
            query_filter = None
            if module_filter:
                query_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="module",
                            match=models.MatchValue(value=module_filter),
                        )
                    ]
                )

            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                query_filter=query_filter,
                limit=limit,
                with_payload=True,
            )

            self.circuit_breaker.record_success()

            # Return the points directly (they have .score and .payload attributes)
            return results.points

        except Exception as e:
            self.circuit_breaker.record_failure()
            raise e

    def get_collection_info(self) -> dict[str, Any]:
        """Get collection statistics."""
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "collection_name": self.collection_name,
                "vectors_count": getattr(info, 'vectors_count', info.points_count if hasattr(info, 'points_count') else 0),
                "points_count": getattr(info, 'points_count', 0),
                "status": info.status if hasattr(info, 'status') else "unknown",
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"error": str(e)}

    def delete_collection(self) -> None:
        """Delete the collection (for re-indexing)."""
        if self.collection_exists():
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")


@lru_cache
def get_vector_store() -> VectorStore:
    """Get cached vector store instance."""
    return VectorStore()
