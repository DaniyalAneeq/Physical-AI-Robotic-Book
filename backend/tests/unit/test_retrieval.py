"""Unit tests for RetrievalService - vector search, filtering, and empty results."""

import pytest
from unittest.mock import Mock, MagicMock, patch, AsyncMock
from qdrant_client.http.models import ScoredPoint

from src.services.retrieval import RetrievalService
from src.models.session import ScopeContext, ScopeType
from src.models.chunk import ChunkPayload


@pytest.fixture
def retrieval_service():
    """Create a RetrievalService instance with mocked Qdrant client."""
    service = RetrievalService()
    service._client = Mock()
    return service


@pytest.fixture
def sample_payload():
    """Sample chunk payload for testing."""
    return {
        "module_id": "module-1",
        "chapter_id": "chapter-3",
        "section_id": "section-3.2",
        "paragraph_id": 1,
        "content_type": "paragraph",
        "text": "This is sample text about ROS 2 navigation.",
        "file_path": "module-1/chapter-3.md",
        "chapter_title": "ROS 2 Navigation",
        "section_title": "Nav2 Configuration",
    }


@pytest.fixture
def sample_vector():
    """Sample embedding vector."""
    return [0.1] * 1536


class TestRetrievalServiceSearch:
    """Tests for RetrievalService.search() method."""

    @pytest.mark.asyncio
    async def test_search_returns_chunks_with_scores(self, retrieval_service, sample_payload, sample_vector):
        """Test that search returns chunks with relevance scores."""
        # Mock Qdrant response
        mock_point = Mock()
        mock_point.payload = sample_payload
        mock_point.score = 0.92

        mock_result = Mock()
        mock_result.points = [mock_point]

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search
        scope = ScopeContext(type=ScopeType.ALL)
        results = await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify results
        assert len(results) == 1
        chunk, score = results[0]
        assert isinstance(chunk, ChunkPayload)
        assert chunk.module_id == "module-1"
        assert chunk.chapter_id == "chapter-3"
        assert score == 0.92

    @pytest.mark.asyncio
    async def test_search_with_page_scope_filter(self, retrieval_service, sample_payload, sample_vector):
        """Test that page scope applies chapter_id filter."""
        mock_point = Mock()
        mock_point.payload = sample_payload
        mock_point.score = 0.85

        mock_result = Mock()
        mock_result.points = [mock_point]

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search with page scope
        scope = ScopeContext(type=ScopeType.PAGE, chapter_id="chapter-3")
        results = await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify filter was applied
        call_args = retrieval_service.client.query_points.call_args
        assert call_args is not None
        query_filter = call_args.kwargs.get("query_filter")
        assert query_filter is not None
        assert len(query_filter.must) == 1
        assert query_filter.must[0].key == "chapter_id"

    @pytest.mark.asyncio
    async def test_search_with_module_scope_filter(self, retrieval_service, sample_payload, sample_vector):
        """Test that module scope applies module_id filter."""
        mock_point = Mock()
        mock_point.payload = sample_payload
        mock_point.score = 0.78

        mock_result = Mock()
        mock_result.points = [mock_point]

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search with module scope
        scope = ScopeContext(type=ScopeType.MODULE, module_id="module-1")
        results = await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify filter was applied
        call_args = retrieval_service.client.query_points.call_args
        query_filter = call_args.kwargs.get("query_filter")
        assert query_filter is not None
        assert len(query_filter.must) == 1
        assert query_filter.must[0].key == "module_id"

    @pytest.mark.asyncio
    async def test_search_with_all_scope_no_filter(self, retrieval_service, sample_payload, sample_vector):
        """Test that ALL scope does not apply any filter."""
        mock_result = Mock()
        mock_result.points = []

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search with ALL scope
        scope = ScopeContext(type=ScopeType.ALL)
        await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify no filter was applied
        call_args = retrieval_service.client.query_points.call_args
        query_filter = call_args.kwargs.get("query_filter")
        assert query_filter is None

    @pytest.mark.asyncio
    async def test_search_returns_empty_list_when_no_results(self, retrieval_service, sample_vector):
        """Test that search returns empty list when no chunks match."""
        mock_result = Mock()
        mock_result.points = []

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search
        scope = ScopeContext(type=ScopeType.ALL)
        results = await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify empty results
        assert results == []

    @pytest.mark.asyncio
    async def test_search_respects_limit_parameter(self, retrieval_service, sample_vector):
        """Test that search respects the limit parameter."""
        mock_result = Mock()
        mock_result.points = []

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search with custom limit
        scope = ScopeContext(type=ScopeType.ALL)
        await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=10,
        )

        # Verify limit was passed
        call_args = retrieval_service.client.query_points.call_args
        assert call_args.kwargs.get("limit") == 10

    @pytest.mark.asyncio
    async def test_search_selection_scope_no_filter(self, retrieval_service, sample_vector):
        """Test that selection scope does not apply metadata filter (uses embedding instead)."""
        mock_result = Mock()
        mock_result.points = []

        retrieval_service.client.query_points = Mock(return_value=mock_result)

        # Execute search with selection scope
        scope = ScopeContext(type=ScopeType.SELECTION, selected_text="some selected text")
        await retrieval_service.search(
            query_vector=sample_vector,
            scope=scope,
            limit=5,
        )

        # Verify no filter was applied (selection uses embedding similarity)
        call_args = retrieval_service.client.query_points.call_args
        query_filter = call_args.kwargs.get("query_filter")
        assert query_filter is None


class TestRetrievalServiceConnection:
    """Tests for RetrievalService connection validation."""

    @pytest.mark.asyncio
    async def test_validate_connection_success(self, retrieval_service):
        """Test connection validation returns True on success."""
        retrieval_service.client.get_collections = Mock(return_value=Mock(collections=[]))

        result = await retrieval_service.validate_connection()

        assert result is True

    @pytest.mark.asyncio
    async def test_validate_connection_failure(self, retrieval_service):
        """Test connection validation returns False on failure."""
        retrieval_service.client.get_collections = Mock(side_effect=Exception("Connection error"))

        result = await retrieval_service.validate_connection()

        assert result is False


class TestRetrievalServiceCollection:
    """Tests for RetrievalService collection management."""

    @pytest.mark.asyncio
    async def test_ensure_collection_creates_if_not_exists(self, retrieval_service):
        """Test that ensure_collection creates collection if it doesn't exist."""
        mock_collections = Mock()
        mock_collections.collections = []
        retrieval_service.client.get_collections = Mock(return_value=mock_collections)
        retrieval_service.client.create_collection = Mock()

        await retrieval_service.ensure_collection()

        retrieval_service.client.create_collection.assert_called_once()

    @pytest.mark.asyncio
    async def test_ensure_collection_skips_if_exists(self, retrieval_service):
        """Test that ensure_collection skips creation if collection exists."""
        mock_collection = Mock()
        mock_collection.name = "textbook_content"
        mock_collections = Mock()
        mock_collections.collections = [mock_collection]
        retrieval_service.client.get_collections = Mock(return_value=mock_collections)
        retrieval_service.client.create_collection = Mock()

        await retrieval_service.ensure_collection()

        retrieval_service.client.create_collection.assert_not_called()
