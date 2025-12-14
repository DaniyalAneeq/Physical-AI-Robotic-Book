"""RAG retrieval tool for textbook content using Qdrant vector search."""

import logging

from app.services.embeddings import EmbeddingService
from app.services.vector_store import VectorStore

logger = logging.getLogger(__name__)


async def retrieve_textbook_content(
    query: str,
    module_filter: str | None = None,
    limit: int = 5,
) -> str:
    """
    Retrieve relevant textbook content chunks for a given query.

    This function is designed to be used as a tool by the OpenAI agent.
    It performs semantic search over the textbook content and returns
    relevant passages with citation information.

    Args:
        query: User's question or search query
        module_filter: Optional module name to filter results (e.g., "module-1")
        limit: Maximum number of chunks to retrieve (default: 5)

    Returns:
        Formatted string with retrieved content and citations
    """
    try:
        # Initialize services
        embedding_service = EmbeddingService()
        vector_store = VectorStore()

        # Generate query embedding
        query_vector = await embedding_service.embed_text(query)

        # Search Qdrant with optional module filter
        results = vector_store.search(
            query_vector=query_vector,
            module_filter=module_filter,
            limit=limit,
        )

        if not results:
            return "No relevant content found in the textbook for this query."

        # Format results with citations
        formatted_results = []
        for i, result in enumerate(results, 1):
            payload = result.payload
            score = result.score

            citation_info = []
            if payload.get("module"):
                citation_info.append(f"Module: {payload['module']}")
            if payload.get("chapter"):
                citation_info.append(f"Chapter: {payload['chapter']}")
            if payload.get("section"):
                citation_info.append(f"Section: {payload['section']}")

            citation_str = " | ".join(citation_info) if citation_info else "Unknown source"

            formatted_results.append(
                f"[{i}] {citation_str} (relevance: {score:.2f})\n"
                f"{payload.get('text', 'No content available')}\n"
            )

        return "\n---\n".join(formatted_results)

    except Exception as e:
        logger.error(f"Error retrieving textbook content: {e}", exc_info=True)
        return f"Error retrieving content: {str(e)}"
