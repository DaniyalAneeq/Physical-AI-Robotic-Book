#!/usr/bin/env python
"""Initialize Qdrant collection with proper configuration."""

import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.vector_store import get_vector_store

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def main():
    """Initialize Qdrant collection."""
    logger.info("Initializing Qdrant collection...")

    store = get_vector_store()

    # Check if collection already exists
    if store.collection_exists():
        logger.info(f"Collection '{store.collection_name}' already exists")
        info = store.get_collection_info()
        logger.info(f"Collection info: {info}")

        # Ask user if they want to recreate
        response = input("Do you want to delete and recreate the collection? (y/N): ")
        if response.lower() == "y":
            store.delete_collection()
            store.create_collection()
            logger.info("Collection recreated successfully")
        else:
            logger.info("Keeping existing collection")
    else:
        store.create_collection()
        logger.info("Collection created successfully")

    # Final status
    info = store.get_collection_info()
    logger.info(f"Final collection status: {info}")


if __name__ == "__main__":
    main()
