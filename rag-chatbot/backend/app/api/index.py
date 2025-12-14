"""Index management API endpoints for managing vector database."""

import logging

from fastapi import APIRouter, HTTPException

from app.services.vector_store import VectorStore

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("/index/status")
async def get_index_status():
    """
    Get current index status including collection info and document count.

    Returns:
        dict: Index status with collection name, count, and vector dimensions
    """
    try:
        vector_store = VectorStore()
        collection_info = vector_store.get_collection_info()

        return {
            "status": "ready",
            "collection": collection_info.get("collection_name"),
            "document_count": collection_info.get("vectors_count", 0),
            "indexed_modules": collection_info.get("indexed_modules", []),
        }
    except Exception as e:
        logger.error(f"Failed to get index status: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve index status") from e


@router.post("/index/rebuild")
async def rebuild_index():
    """
    Trigger a full index rebuild.

    This endpoint should be called manually or via admin interface.
    It will re-index all textbook content from the configured docs path.

    Returns:
        dict: Rebuild status message

    Note:
        In production, this should be an async background task with job tracking.
    """
    try:
        logger.info("Index rebuild requested")

        # TODO: Implement background task for async indexing
        # For now, return message indicating manual rebuild required
        return {
            "status": "pending",
            "message": "Index rebuild must be triggered manually via index_textbook.py script",
            "command": "uv run python scripts/index_textbook.py",
        }
    except Exception as e:
        logger.error(f"Failed to trigger index rebuild: {e}")
        raise HTTPException(status_code=500, detail="Failed to trigger rebuild") from e
