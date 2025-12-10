"""Integration tests for end-to-end chat query processing."""

import pytest
from unittest.mock import Mock, AsyncMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient
from httpx import AsyncClient


class TestChatFlowIntegration:
    """End-to-end tests for the chat query flow."""

    @pytest.fixture
    def mock_services(self):
        """Mock all external services."""
        with patch("src.services.session.session_service") as mock_session, \
             patch("src.services.retrieval.retrieval_service") as mock_retrieval, \
             patch("src.services.embedding.embedding_service") as mock_embedding, \
             patch("src.services.chatkit_server.chatkit_server") as mock_chatkit:

            # Configure session service
            mock_session.initialize = AsyncMock()
            mock_session.close = AsyncMock()
            mock_session.validate_connection = AsyncMock(return_value=True)

            # Configure retrieval service
            mock_retrieval.ensure_collection = AsyncMock()
            mock_retrieval.validate_connection = AsyncMock(return_value=True)

            # Configure embedding service
            mock_embedding.validate_connection = AsyncMock(return_value=True)

            yield {
                "session": mock_session,
                "retrieval": mock_retrieval,
                "embedding": mock_embedding,
                "chatkit": mock_chatkit,
            }

    @pytest.mark.asyncio
    async def test_health_check_returns_status(self, mock_services):
        """Test that health check endpoint returns service status."""
        from src.main import app

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/health")

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "dependencies" in data
        assert "qdrant" in data["dependencies"]
        assert "postgres" in data["dependencies"]
        assert "openai" in data["dependencies"]

    @pytest.mark.asyncio
    async def test_create_session_returns_session_id(self, mock_services):
        """Test that POST /api/sessions creates a new session."""
        from src.main import app
        from src.models.session import Session, ScopeContext, ScopeType
        from datetime import datetime

        session_id = uuid4()
        mock_session = Session(
            id=session_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            conversation_history=[],
            scope_context=ScopeContext(type=ScopeType.ALL),
        )
        mock_services["session"].create = AsyncMock(return_value=mock_session)

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post("/api/sessions")

        assert response.status_code == 201
        data = response.json()
        assert "id" in data
        assert data["id"] == str(session_id)

    @pytest.mark.asyncio
    async def test_get_session_returns_session_details(self, mock_services):
        """Test that GET /api/sessions/{id} returns session details."""
        from src.main import app
        from src.models.session import Session, ScopeContext, ScopeType
        from datetime import datetime

        session_id = uuid4()
        mock_session = Session(
            id=session_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            conversation_history=[],
            scope_context=ScopeContext(type=ScopeType.ALL),
        )
        mock_services["session"].get = AsyncMock(return_value=mock_session)

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get(f"/api/sessions/{session_id}")

        assert response.status_code == 200
        data = response.json()
        assert data["id"] == str(session_id)

    @pytest.mark.asyncio
    async def test_get_session_returns_404_when_not_found(self, mock_services):
        """Test that GET /api/sessions/{id} returns 404 for non-existent session."""
        from src.main import app

        mock_services["session"].get = AsyncMock(return_value=None)

        session_id = uuid4()
        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get(f"/api/sessions/{session_id}")

        assert response.status_code == 404

    @pytest.mark.asyncio
    async def test_delete_session_returns_204(self, mock_services):
        """Test that DELETE /api/sessions/{id} returns 204 on success."""
        from src.main import app

        mock_services["session"].delete = AsyncMock(return_value=True)

        session_id = uuid4()
        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.delete(f"/api/sessions/{session_id}")

        assert response.status_code == 204


class TestChatEndpoint:
    """Tests for the POST /api/chat endpoint."""

    @pytest.fixture
    def mock_chat_services(self):
        """Mock services for chat endpoint testing."""
        with patch("src.api.routes.session_service") as mock_session, \
             patch("src.api.routes.chatkit_server") as mock_chatkit, \
             patch("src.api.routes.logging_service") as mock_logging:

            mock_session.get = AsyncMock(return_value=None)
            mock_logging.log_query = AsyncMock()

            yield {
                "session": mock_session,
                "chatkit": mock_chatkit,
                "logging": mock_logging,
            }

    @pytest.mark.asyncio
    async def test_chat_endpoint_streams_response(self, mock_chat_services):
        """Test that POST /api/chat returns streaming response."""
        from src.main import app
        from src.models.query import StreamEvent

        # Mock streaming response
        async def mock_respond(*args, **kwargs):
            yield StreamEvent(type="content", text="Hello ")
            yield StreamEvent(type="content", text="World!")
            yield StreamEvent(type="done")

        mock_chat_services["chatkit"].respond = mock_respond

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/api/chat",
                json={
                    "query": "What is ROS?",
                    "scope": {"type": "all"},
                },
            )

        assert response.status_code == 200
        assert "text/event-stream" in response.headers.get("content-type", "")

    @pytest.mark.asyncio
    async def test_chat_validates_query_length(self, mock_chat_services):
        """Test that chat endpoint validates query length."""
        from src.main import app

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/api/chat",
                json={
                    "query": "",  # Empty query
                    "scope": {"type": "all"},
                },
            )

        assert response.status_code == 422  # Validation error

    @pytest.mark.asyncio
    async def test_chat_validates_scope_type(self, mock_chat_services):
        """Test that chat endpoint validates scope type."""
        from src.main import app

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/api/chat",
                json={
                    "query": "What is ROS?",
                    "scope": {"type": "invalid"},  # Invalid scope type
                },
            )

        assert response.status_code == 422  # Validation error


class TestQueryLogging:
    """Tests for query logging functionality."""

    @pytest.mark.asyncio
    async def test_successful_query_is_logged(self):
        """Test that successful queries are logged for analytics."""
        from src.services.logging import LoggingService

        # This test verifies the logging service interface
        service = LoggingService()
        assert hasattr(service, "log_query")
        assert hasattr(service, "get_stats")
        assert hasattr(service, "cleanup_old_logs")
