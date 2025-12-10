"""Unit tests for SessionService - session lifecycle and expiration."""

import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from datetime import datetime, timedelta
from uuid import uuid4, UUID
import json

from src.services.session import SessionService
from src.models.session import Session, ScopeContext, ScopeType, ConversationEntry


@pytest.fixture
def session_service():
    """Create a SessionService instance with mocked pool."""
    service = SessionService()
    service._pool = AsyncMock()
    return service


@pytest.fixture
def sample_session_id():
    """Sample session UUID."""
    return uuid4()


@pytest.fixture
def sample_session_row(sample_session_id):
    """Sample database row for session."""
    return {
        "id": str(sample_session_id),
        "created_at": datetime.utcnow(),
        "last_activity": datetime.utcnow(),
        "conversation_history": json.dumps([]),
        "scope_context": json.dumps({"type": "all"}),
    }


class TestSessionServiceCreate:
    """Tests for SessionService.create() method."""

    @pytest.mark.asyncio
    async def test_create_session_returns_session_object(self, session_service):
        """Test that create returns a valid Session object."""
        # Mock connection
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        session_service._pool.connection = Mock(return_value=mock_conn)

        # Create session
        session = await session_service.create()

        # Verify session object
        assert isinstance(session, Session)
        assert isinstance(session.id, UUID)
        assert session.conversation_history == []
        assert session.scope_context.type == ScopeType.ALL

    @pytest.mark.asyncio
    async def test_create_session_with_custom_scope(self, session_service):
        """Test that create accepts custom scope context."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        session_service._pool.connection = Mock(return_value=mock_conn)

        scope = ScopeContext(type=ScopeType.PAGE, chapter_id="chapter-3")
        session = await session_service.create(scope_context=scope)

        assert session.scope_context.type == ScopeType.PAGE
        assert session.scope_context.chapter_id == "chapter-3"

    @pytest.mark.asyncio
    async def test_create_session_executes_insert(self, session_service):
        """Test that create executes INSERT statement."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        mock_conn.execute = AsyncMock()
        session_service._pool.connection = Mock(return_value=mock_conn)

        await session_service.create()

        mock_conn.execute.assert_called_once()
        call_args = mock_conn.execute.call_args[0]
        assert "INSERT INTO sessions" in call_args[0]


class TestSessionServiceGet:
    """Tests for SessionService.get() method."""

    @pytest.mark.asyncio
    async def test_get_session_returns_session_when_found(self, session_service, sample_session_id, sample_session_row):
        """Test that get returns Session when found."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_cursor = AsyncMock()
        mock_cursor.__aenter__ = AsyncMock(return_value=mock_cursor)
        mock_cursor.__aexit__ = AsyncMock()
        mock_cursor.fetchone = AsyncMock(return_value=sample_session_row)

        mock_conn.cursor = Mock(return_value=mock_cursor)
        session_service._pool.connection = Mock(return_value=mock_conn)

        session = await session_service.get(sample_session_id)

        assert session is not None
        assert str(session.id) == str(sample_session_id)

    @pytest.mark.asyncio
    async def test_get_session_returns_none_when_not_found(self, session_service, sample_session_id):
        """Test that get returns None when session not found."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_cursor = AsyncMock()
        mock_cursor.__aenter__ = AsyncMock(return_value=mock_cursor)
        mock_cursor.__aexit__ = AsyncMock()
        mock_cursor.fetchone = AsyncMock(return_value=None)

        mock_conn.cursor = Mock(return_value=mock_cursor)
        session_service._pool.connection = Mock(return_value=mock_conn)

        session = await session_service.get(sample_session_id)

        assert session is None

    @pytest.mark.asyncio
    async def test_get_session_parses_conversation_history(self, session_service, sample_session_id):
        """Test that get correctly parses conversation history JSON."""
        history = [
            {"query": "What is ROS?", "response": "ROS is...", "timestamp": "2025-12-10T10:00:00"},
            {"query": "How to use it?", "response": "You can...", "timestamp": "2025-12-10T10:01:00"},
        ]
        row = {
            "id": str(sample_session_id),
            "created_at": datetime.utcnow(),
            "last_activity": datetime.utcnow(),
            "conversation_history": json.dumps(history),
            "scope_context": json.dumps({"type": "all"}),
        }

        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_cursor = AsyncMock()
        mock_cursor.__aenter__ = AsyncMock(return_value=mock_cursor)
        mock_cursor.__aexit__ = AsyncMock()
        mock_cursor.fetchone = AsyncMock(return_value=row)

        mock_conn.cursor = Mock(return_value=mock_cursor)
        session_service._pool.connection = Mock(return_value=mock_conn)

        session = await session_service.get(sample_session_id)

        assert len(session.conversation_history) == 2
        assert session.conversation_history[0].query == "What is ROS?"


class TestSessionServiceUpdate:
    """Tests for SessionService.update() method."""

    @pytest.mark.asyncio
    async def test_update_appends_conversation_entry(self, session_service, sample_session_id, sample_session_row):
        """Test that update appends new conversation entry."""
        # Mock get to return existing session
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        mock_conn.execute = AsyncMock()

        mock_cursor = AsyncMock()
        mock_cursor.__aenter__ = AsyncMock(return_value=mock_cursor)
        mock_cursor.__aexit__ = AsyncMock()
        mock_cursor.fetchone = AsyncMock(return_value=sample_session_row)

        mock_conn.cursor = Mock(return_value=mock_cursor)
        session_service._pool.connection = Mock(return_value=mock_conn)

        updated = await session_service.update(
            session_id=sample_session_id,
            query="What is ROS?",
            response="ROS is a robotics middleware...",
        )

        assert updated is not None
        assert len(updated.conversation_history) == 1
        assert updated.conversation_history[0].query == "What is ROS?"

    @pytest.mark.asyncio
    async def test_update_returns_none_if_session_not_found(self, session_service, sample_session_id):
        """Test that update returns None if session doesn't exist."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_cursor = AsyncMock()
        mock_cursor.__aenter__ = AsyncMock(return_value=mock_cursor)
        mock_cursor.__aexit__ = AsyncMock()
        mock_cursor.fetchone = AsyncMock(return_value=None)

        mock_conn.cursor = Mock(return_value=mock_cursor)
        session_service._pool.connection = Mock(return_value=mock_conn)

        updated = await session_service.update(
            session_id=sample_session_id,
            query="test",
            response="test",
        )

        assert updated is None


class TestSessionServiceDelete:
    """Tests for SessionService.delete() method."""

    @pytest.mark.asyncio
    async def test_delete_returns_true_when_deleted(self, session_service, sample_session_id):
        """Test that delete returns True when session is deleted."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_result = Mock()
        mock_result.rowcount = 1
        mock_conn.execute = AsyncMock(return_value=mock_result)

        session_service._pool.connection = Mock(return_value=mock_conn)

        result = await session_service.delete(sample_session_id)

        assert result is True

    @pytest.mark.asyncio
    async def test_delete_returns_false_when_not_found(self, session_service, sample_session_id):
        """Test that delete returns False when session not found."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_result = Mock()
        mock_result.rowcount = 0
        mock_conn.execute = AsyncMock(return_value=mock_result)

        session_service._pool.connection = Mock(return_value=mock_conn)

        result = await session_service.delete(sample_session_id)

        assert result is False


class TestSessionServiceCleanup:
    """Tests for SessionService.cleanup() method."""

    @pytest.mark.asyncio
    async def test_cleanup_deletes_expired_sessions(self, session_service):
        """Test that cleanup deletes sessions older than 24 hours."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()

        mock_result = Mock()
        mock_result.rowcount = 5
        mock_conn.execute = AsyncMock(return_value=mock_result)

        session_service._pool.connection = Mock(return_value=mock_conn)

        deleted_count = await session_service.cleanup()

        assert deleted_count == 5
        mock_conn.execute.assert_called_once()
        call_args = mock_conn.execute.call_args[0]
        assert "DELETE FROM sessions" in call_args[0]
        assert "last_activity" in call_args[0]


class TestSessionServiceConnection:
    """Tests for SessionService connection validation."""

    @pytest.mark.asyncio
    async def test_validate_connection_success(self, session_service):
        """Test connection validation returns True on success."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        mock_conn.execute = AsyncMock()

        session_service._pool.connection = Mock(return_value=mock_conn)

        result = await session_service.validate_connection()

        assert result is True

    @pytest.mark.asyncio
    async def test_validate_connection_failure(self, session_service):
        """Test connection validation returns False on failure."""
        mock_conn = AsyncMock()
        mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
        mock_conn.__aexit__ = AsyncMock()
        mock_conn.execute = AsyncMock(side_effect=Exception("Connection error"))

        session_service._pool.connection = Mock(return_value=mock_conn)

        result = await session_service.validate_connection()

        assert result is False
