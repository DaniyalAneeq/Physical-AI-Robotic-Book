"""Session and ScopeContext models for conversation management."""

from datetime import datetime
from enum import Enum
from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID


class ScopeType(str, Enum):
    """Type of content scope for queries."""
    PAGE = "page"
    SELECTION = "selection"
    MODULE = "module"
    ALL = "all"


class ScopeContext(BaseModel):
    """Defines the boundaries for content retrieval."""
    type: ScopeType = Field(..., description="Scope type")
    chapter_id: Optional[str] = Field(None, description="Chapter filter (required if type='page')")
    module_id: Optional[str] = Field(None, description="Module filter (required if type='module')")
    selected_text: Optional[str] = Field(None, max_length=5000, description="User-highlighted text (required if type='selection')")


class ConversationEntry(BaseModel):
    """A single Q&A pair in conversation history."""
    query: str = Field(..., description="User's question")
    response: str = Field(..., description="Chatbot's answer")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Entry timestamp")


class Session(BaseModel):
    """A user's conversation context for multi-turn interactions.

    Stored in: Neon Serverless Postgres
    """
    id: UUID = Field(..., description="Unique session identifier")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session creation time (UTC)")
    last_activity: datetime = Field(default_factory=datetime.utcnow, description="Last query timestamp (UTC)")
    conversation_history: list[ConversationEntry] = Field(default_factory=list, max_length=20, description="Previous Q&A pairs (max 20)")
    scope_context: ScopeContext = Field(default_factory=lambda: ScopeContext(type=ScopeType.ALL), description="Current scope selection")

    class Config:
        from_attributes = True


class SessionCreate(BaseModel):
    """Request model for creating a new session."""
    scope_context: Optional[ScopeContext] = None


class SessionResponse(BaseModel):
    """Response model for session endpoints."""
    id: str = Field(..., description="Unique session identifier")
    created_at: datetime = Field(..., description="Session creation timestamp")
    last_activity: datetime = Field(..., description="Last query timestamp")
    conversation_history: list[ConversationEntry] = Field(default_factory=list)
    scope_context: dict = Field(default_factory=dict)

    @classmethod
    def from_session(cls, session: Session) -> "SessionResponse":
        """Create response from Session model."""
        return cls(
            id=str(session.id),
            created_at=session.created_at,
            last_activity=session.last_activity,
            conversation_history=session.conversation_history,
            scope_context=session.scope_context.model_dump() if session.scope_context else {},
        )
