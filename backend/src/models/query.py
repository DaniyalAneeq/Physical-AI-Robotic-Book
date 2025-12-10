"""Query request/response models matching OpenAPI spec."""

from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID

from .session import ScopeContext


class ChatRequest(BaseModel):
    """Request model for POST /api/chat endpoint."""
    session_id: Optional[UUID] = Field(None, description="Session identifier (optional, creates new session if not provided)")
    query: str = Field(..., min_length=1, max_length=1000, description="User's natural language question")
    scope: ScopeContext = Field(..., description="Retrieval scope")


class Citation(BaseModel):
    """A reference to source content in the textbook."""
    chunk_id: str = Field(..., description="Reference to ContentChunk")
    chapter_title: str = Field(..., description="Human-readable chapter name")
    section_title: str = Field(..., description="Human-readable section name")
    paragraph_number: int = Field(..., ge=1, description="Paragraph within section")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score from Qdrant")
    excerpt: str = Field(..., max_length=200, description="Short text excerpt")


class ChatResponse(BaseModel):
    """Response model for chat endpoint (non-streaming)."""
    session_id: str = Field(..., description="Session identifier")
    content: str = Field(..., description="Generated answer text (Markdown formatted)")
    citations: list[Citation] = Field(..., min_length=1, description="Source references")
    latency_ms: int = Field(..., description="Response generation time")


class StreamEvent(BaseModel):
    """A single event in the SSE stream."""
    type: str = Field(..., description="Event type: 'content', 'citation', 'done', 'error'")
    text: Optional[str] = Field(None, description="Text content (for type='content')")
    citation: Optional[Citation] = Field(None, description="Citation data (for type='citation')")
    error: Optional[str] = Field(None, description="Error message (for type='error')")


class Error(BaseModel):
    """Error response model."""
    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    retry_after: Optional[int] = Field(None, description="Seconds to wait before retry (for rate limits)")


class HealthStatus(BaseModel):
    """Health check response model."""
    status: str = Field(..., description="Overall service status: 'healthy' or 'unhealthy'")
    version: str = Field(..., description="API version")
    dependencies: dict = Field(..., description="Status of each dependency")
