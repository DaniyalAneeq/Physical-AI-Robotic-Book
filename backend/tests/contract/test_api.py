"""Contract tests for OpenAPI schema validation."""

import pytest
from pydantic import ValidationError
from uuid import uuid4
from datetime import datetime

from src.models.query import (
    ChatRequest,
    ChatResponse,
    Citation,
    StreamEvent,
    Error,
    HealthStatus,
)
from src.models.session import (
    Session,
    SessionCreate,
    SessionResponse,
    ScopeContext,
    ScopeType,
    ConversationEntry,
)


class TestChatRequestSchema:
    """Tests for ChatRequest schema validation per openapi.yaml."""

    def test_chat_request_with_all_fields(self):
        """Test ChatRequest accepts all valid fields."""
        request = ChatRequest(
            session_id=uuid4(),
            query="What is ROS 2?",
            scope=ScopeContext(type=ScopeType.ALL),
        )
        assert request.query == "What is ROS 2?"
        assert request.scope.type == ScopeType.ALL

    def test_chat_request_without_session_id(self):
        """Test ChatRequest allows optional session_id."""
        request = ChatRequest(
            query="What is ROS 2?",
            scope=ScopeContext(type=ScopeType.ALL),
        )
        assert request.session_id is None

    def test_chat_request_rejects_empty_query(self):
        """Test ChatRequest rejects empty query."""
        with pytest.raises(ValidationError):
            ChatRequest(
                query="",
                scope=ScopeContext(type=ScopeType.ALL),
            )

    def test_chat_request_rejects_query_too_long(self):
        """Test ChatRequest rejects query over 1000 characters."""
        with pytest.raises(ValidationError):
            ChatRequest(
                query="x" * 1001,
                scope=ScopeContext(type=ScopeType.ALL),
            )

    def test_chat_request_with_page_scope(self):
        """Test ChatRequest with page scope includes chapter_id."""
        request = ChatRequest(
            query="Explain this section",
            scope=ScopeContext(type=ScopeType.PAGE, chapter_id="chapter-3"),
        )
        assert request.scope.type == ScopeType.PAGE
        assert request.scope.chapter_id == "chapter-3"

    def test_chat_request_with_selection_scope(self):
        """Test ChatRequest with selection scope includes selected_text."""
        request = ChatRequest(
            query="What does this mean?",
            scope=ScopeContext(
                type=ScopeType.SELECTION,
                selected_text="nav2_params = {...}",
            ),
        )
        assert request.scope.type == ScopeType.SELECTION
        assert request.scope.selected_text == "nav2_params = {...}"

    def test_chat_request_with_module_scope(self):
        """Test ChatRequest with module scope includes module_id."""
        request = ChatRequest(
            query="Overview of this module",
            scope=ScopeContext(type=ScopeType.MODULE, module_id="module-2"),
        )
        assert request.scope.type == ScopeType.MODULE
        assert request.scope.module_id == "module-2"


class TestScopeContextSchema:
    """Tests for ScopeContext schema validation."""

    def test_scope_all_type(self):
        """Test ScopeContext with 'all' type."""
        scope = ScopeContext(type=ScopeType.ALL)
        assert scope.type == ScopeType.ALL

    def test_scope_page_type(self):
        """Test ScopeContext with 'page' type."""
        scope = ScopeContext(type=ScopeType.PAGE, chapter_id="chapter-1")
        assert scope.type == ScopeType.PAGE
        assert scope.chapter_id == "chapter-1"

    def test_scope_selection_type(self):
        """Test ScopeContext with 'selection' type."""
        scope = ScopeContext(
            type=ScopeType.SELECTION,
            selected_text="Some highlighted text",
        )
        assert scope.type == ScopeType.SELECTION
        assert scope.selected_text == "Some highlighted text"

    def test_scope_module_type(self):
        """Test ScopeContext with 'module' type."""
        scope = ScopeContext(type=ScopeType.MODULE, module_id="module-3")
        assert scope.type == ScopeType.MODULE
        assert scope.module_id == "module-3"


class TestCitationSchema:
    """Tests for Citation schema validation."""

    def test_citation_with_all_fields(self):
        """Test Citation accepts all required fields."""
        citation = Citation(
            chunk_id="abc-123",
            chapter_title="ROS 2 Navigation",
            section_title="Nav2 Setup",
            paragraph_number=4,
            relevance_score=0.92,
            excerpt="The navigation stack requires...",
        )
        assert citation.chunk_id == "abc-123"
        assert citation.relevance_score == 0.92

    def test_citation_rejects_invalid_relevance_score(self):
        """Test Citation rejects relevance_score outside 0-1 range."""
        with pytest.raises(ValidationError):
            Citation(
                chunk_id="abc-123",
                chapter_title="Test",
                section_title="Test",
                paragraph_number=1,
                relevance_score=1.5,  # Invalid: > 1.0
                excerpt="Test",
            )

    def test_citation_rejects_negative_paragraph_number(self):
        """Test Citation rejects paragraph_number < 1."""
        with pytest.raises(ValidationError):
            Citation(
                chunk_id="abc-123",
                chapter_title="Test",
                section_title="Test",
                paragraph_number=0,  # Invalid: < 1
                relevance_score=0.5,
                excerpt="Test",
            )


class TestStreamEventSchema:
    """Tests for StreamEvent schema validation."""

    def test_stream_event_content_type(self):
        """Test StreamEvent with content type."""
        event = StreamEvent(type="content", text="Hello world")
        assert event.type == "content"
        assert event.text == "Hello world"

    def test_stream_event_citation_type(self):
        """Test StreamEvent with citation type."""
        citation = Citation(
            chunk_id="abc-123",
            chapter_title="Test",
            section_title="Test",
            paragraph_number=1,
            relevance_score=0.9,
            excerpt="Test excerpt",
        )
        event = StreamEvent(type="citation", citation=citation)
        assert event.type == "citation"
        assert event.citation.chunk_id == "abc-123"

    def test_stream_event_done_type(self):
        """Test StreamEvent with done type."""
        event = StreamEvent(type="done")
        assert event.type == "done"

    def test_stream_event_error_type(self):
        """Test StreamEvent with error type."""
        event = StreamEvent(type="error", error="Something went wrong")
        assert event.type == "error"
        assert event.error == "Something went wrong"


class TestSessionSchema:
    """Tests for Session schema validation."""

    def test_session_with_all_fields(self):
        """Test Session accepts all fields."""
        session = Session(
            id=uuid4(),
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            conversation_history=[],
            scope_context=ScopeContext(type=ScopeType.ALL),
        )
        assert session.conversation_history == []

    def test_session_with_conversation_history(self):
        """Test Session with conversation history."""
        entry = ConversationEntry(
            query="What is ROS?",
            response="ROS is...",
            timestamp=datetime.utcnow(),
        )
        session = Session(
            id=uuid4(),
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            conversation_history=[entry],
            scope_context=ScopeContext(type=ScopeType.ALL),
        )
        assert len(session.conversation_history) == 1


class TestSessionResponseSchema:
    """Tests for SessionResponse schema validation."""

    def test_session_response_from_session(self):
        """Test SessionResponse.from_session() creates valid response."""
        session = Session(
            id=uuid4(),
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            conversation_history=[],
            scope_context=ScopeContext(type=ScopeType.ALL),
        )
        response = SessionResponse.from_session(session)
        assert response.id == str(session.id)


class TestErrorSchema:
    """Tests for Error schema validation."""

    def test_error_with_retry_after(self):
        """Test Error with retry_after field."""
        error = Error(
            error="rate_limit_exceeded",
            message="Rate limit exceeded",
            retry_after=15,
        )
        assert error.retry_after == 15

    def test_error_without_retry_after(self):
        """Test Error without retry_after field."""
        error = Error(
            error="not_found",
            message="Session not found",
        )
        assert error.retry_after is None


class TestHealthStatusSchema:
    """Tests for HealthStatus schema validation."""

    def test_health_status_healthy(self):
        """Test HealthStatus with healthy status."""
        status = HealthStatus(
            status="healthy",
            version="1.0.0",
            dependencies={
                "qdrant": "connected",
                "postgres": "connected",
                "openai": "connected",
            },
        )
        assert status.status == "healthy"

    def test_health_status_unhealthy(self):
        """Test HealthStatus with unhealthy status."""
        status = HealthStatus(
            status="unhealthy",
            version="1.0.0",
            dependencies={
                "qdrant": "disconnected",
                "postgres": "connected",
                "openai": "connected",
            },
        )
        assert status.status == "unhealthy"
