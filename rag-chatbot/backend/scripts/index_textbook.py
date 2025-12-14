#!/usr/bin/env python3
"""Index textbook content into Qdrant vector database."""

import asyncio
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client.models import PointStruct

from app.config import get_settings
from app.services.chunker import chunk_directory
from app.services.embeddings import EmbeddingService
from app.services.vector_store import VectorStore

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

settings = get_settings()


async def index_all():
    """Index all textbook content into Qdrant."""
    logger.info("Starting textbook indexing...")

    # Initialize services
    embedding_service = EmbeddingService()
    vector_store = VectorStore()

    # Get textbook content path
    docs_path = Path(settings.textbook_docs_path).resolve()
    if not docs_path.exists():
        logger.error(f"Textbook docs path not found: {docs_path}")
        return

    logger.info(f"Scanning textbook content at: {docs_path}")

    # Chunk all markdown files
    chunks = chunk_directory(docs_path)
    logger.info(f"Found {len(chunks)} chunks to index")

    if not chunks:
        logger.warning("No chunks found to index!")
        return

    # Generate embeddings and create points
    points = []
    for i, chunk in enumerate(chunks):
        if i % 10 == 0:
            logger.info(f"Processing chunk {i+1}/{len(chunks)}")

        # Generate embedding for chunk text
        vector = await embedding_service.embed_text(chunk["text"])

        # Create point with vector and metadata payload
        point = PointStruct(
            id=chunk["id"],
            vector=vector,
            payload={
                "text": chunk["text"],
                "module": chunk["module"],
                "chapter": chunk["chapter"],
                "section": chunk["section"],
                "file_path": chunk["file_path"],
                "char_count": chunk["char_count"],
            },
        )
        points.append(point)

    # Batch upsert to Qdrant
    logger.info(f"Upserting {len(points)} points to Qdrant...")
    vector_store.upsert_points(points)

    logger.info("✅ Indexing complete!")
    logger.info(f"Total chunks indexed: {len(points)}")


if __name__ == "__main__":
    asyncio.run(index_all())
