"""Qdrant vector store service for content retrieval."""

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    Filter,
    FieldCondition,
    MatchValue,
    PointStruct,
)
from typing import Optional

from ..config import get_settings
from ..models.chunk import ChunkPayload
from ..models.session import ScopeContext, ScopeType

settings = get_settings()


class RetrievalService:
    """Service for retrieving content from Qdrant vector store."""

    def __init__(self):
        """Initialize Qdrant client."""
        self._client: Optional[QdrantClient] = None

    @property
    def client(self) -> QdrantClient:
        """Lazy initialization of Qdrant client."""
        if self._client is None:
            self._client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
        return self._client

    async def validate_connection(self) -> bool:
        """Validate connection to Qdrant."""
        try:
            collections = self.client.get_collections()
            return True
        except Exception:
            return False

    async def ensure_collection(self) -> None:
        """Ensure the collection exists, create if not."""
        collections = self.client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if settings.qdrant_collection_name not in collection_names:
            self.client.create_collection(
                collection_name=settings.qdrant_collection_name,
                vectors_config=VectorParams(
                    size=settings.embedding_dimensions,
                    distance=Distance.COSINE,
                ),
            )

    def _build_filter(self, scope: ScopeContext) -> Optional[Filter]:
        """Build Qdrant filter from scope context."""
        if scope.type == ScopeType.ALL:
            return None

        conditions = []

        if scope.type == ScopeType.PAGE and scope.chapter_id:
            conditions.append(
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=scope.chapter_id),
                )
            )
        elif scope.type == ScopeType.MODULE and scope.module_id:
            conditions.append(
                FieldCondition(
                    key="module_id",
                    match=MatchValue(value=scope.module_id),
                )
            )
        # For SELECTION type, we don't filter - we'll embed the selected text
        # and find similar chunks across all content

        if not conditions:
            return None

        return Filter(must=conditions)

    async def search(
        self,
        query_vector: list[float],
        scope: ScopeContext,
        limit: int = 5,
    ) -> list[tuple[ChunkPayload, float]]:
        """Search for relevant content chunks.

        Args:
            query_vector: Embedding vector for the query
            scope: Scope context for filtering
            limit: Maximum number of results

        Returns:
            List of (ChunkPayload, relevance_score) tuples
        """
        query_filter = self._build_filter(scope)

        results = self.client.query_points(
            collection_name=settings.qdrant_collection_name,
            query=query_vector,
            query_filter=query_filter,
            limit=limit,
            with_payload=True,
        )

        chunks = []
        for point in results.points:
            payload = ChunkPayload(**point.payload)
            score = point.score
            chunks.append((payload, score))

        return chunks

    async def upsert_chunks(
        self,
        chunks: list[tuple[str, list[float], ChunkPayload]],
    ) -> None:
        """Upsert content chunks to Qdrant.

        Args:
            chunks: List of (id, vector, payload) tuples
        """
        points = [
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload=payload.model_dump(),
            )
            for chunk_id, vector, payload in chunks
        ]

        self.client.upsert(
            collection_name=settings.qdrant_collection_name,
            points=points,
        )

    async def delete_all(self) -> None:
        """Delete all points in the collection (for re-embedding)."""
        try:
            self.client.delete_collection(settings.qdrant_collection_name)
            await self.ensure_collection()
        except Exception:
            # Collection might not exist
            await self.ensure_collection()

    async def get_collection_info(self) -> dict:
        """Get information about the collection."""
        try:
            info = self.client.get_collection(settings.qdrant_collection_name)
            return {
                "name": info.config.params.vectors.size if info.config else None,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            return {"error": str(e)}


# Singleton instance
retrieval_service = RetrievalService()
