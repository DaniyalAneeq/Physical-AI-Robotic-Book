"""Session management API endpoints for frontend compatibility."""

import json
import logging
import uuid
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from sse_starlette.sse import EventSourceResponse
from sqlalchemy.orm import Session

from app.models.conversation import Conversation
from app.models.database import get_db_context
from app.models.message import Message
from app.services.agent import get_agent_response_stream

# Import auth dependencies
try:
    from auth_backend.api.deps import get_current_user_optional
    from auth_backend.models.user import User as _User
    AUTH_AVAILABLE = True
except ImportError:
    get_current_user_optional = None
    _User = None
    AUTH_AVAILABLE = False

router = APIRouter()
logger = logging.getLogger(__name__)


# Optional auth dependency - allows unauthenticated access
if AUTH_AVAILABLE and get_current_user_optional:
    maybe_get_user = get_current_user_optional
else:
    # Fallback dependency that returns None
    async def maybe_get_user():
        return None


class ScopeContext(BaseModel):
    """Scope context for queries."""
    type: str  # 'page' | 'selection' | 'module' | 'all'
    chapter_id: str | None = None
    module_id: str | None = None
    selected_text: str | None = None


class SessionCreateRequest(BaseModel):
    """Request model for session creation."""
    scope_context: ScopeContext | None = None


class ChatRequest(BaseModel):
    """Request model for chat messages."""
    session_id: str | None = None
    query: str
    scope: ScopeContext


@router.post("/sessions")
async def create_session(
    request: SessionCreateRequest | None = None,
    user=Depends(maybe_get_user),
):
    """
    Create a new chat session.

    **Authentication Required:** Yes (must be authenticated with completed onboarding)

    This endpoint creates a new conversation in the database and returns session info.

    **Errors:**
    - 401 Unauthorized: Not authenticated or invalid session
    - 403 Forbidden: Authenticated but onboarding not completed
    """
    try:
        with get_db_context() as db:
            # Generate a unique session ID (conversation ID)
            session_id = str(uuid.uuid4())

            # Create a new conversation
            conversation = Conversation(
                id=session_id,
                session_id=session_id,  # Use same ID for both
                title="New Chat",
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            )
            db.add(conversation)
            db.commit()

            return {
                "id": session_id,
                "created_at": conversation.created_at.isoformat(),
                "last_activity": conversation.updated_at.isoformat(),
                "conversation_history": [],
                "scope_context": request.scope_context.model_dump() if request and request.scope_context else {"type": "all"},
            }
    except Exception as e:
        logger.error(f"Failed to create session: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to create session")


@router.get("/sessions/{session_id}")
async def get_session(
    session_id: str,
    user=Depends(maybe_get_user),
):
    """
    Get session details by ID.

    **Authentication Required:** Yes (must be authenticated with completed onboarding)

    Returns the conversation and its message history.

    **Errors:**
    - 401 Unauthorized: Not authenticated or invalid session
    - 403 Forbidden: Authenticated but onboarding not completed
    """
    try:
        with get_db_context() as db:
            conversation = db.query(Conversation).filter(Conversation.id == session_id).first()

            if not conversation:
                # Return null instead of 404 to match frontend expectations
                return None

            # Get all messages for this conversation
            messages = (
                db.query(Message)
                .filter(Message.conversation_id == session_id)
                .order_by(Message.created_at.asc())
                .all()
            )

            # Format conversation history
            conversation_history = []
            for msg in messages:
                conversation_history.append({
                    "query": msg.content if msg.role == "user" else "",
                    "response": msg.content if msg.role == "assistant" else "",
                    "timestamp": msg.created_at.isoformat(),
                })

            return {
                "id": str(conversation.id),
                "created_at": conversation.created_at.isoformat(),
                "last_activity": conversation.updated_at.isoformat() if conversation.updated_at else conversation.created_at.isoformat(),
                "conversation_history": conversation_history,
                "scope_context": {"type": "all"},  # Default scope
            }
    except Exception as e:
        logger.error(f"Failed to get session: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to get session")


@router.delete("/sessions/{session_id}")
async def delete_session(
    session_id: str,
    user=Depends(maybe_get_user),
):
    """
    Delete a session and all its messages.

    **Authentication Required:** Yes (must be authenticated with completed onboarding)

    **Errors:**
    - 401 Unauthorized: Not authenticated or invalid session
    - 403 Forbidden: Authenticated but onboarding not completed
    """
    try:
        with get_db_context() as db:
            conversation = db.query(Conversation).filter(Conversation.id == session_id).first()

            if not conversation:
                # Return success even if not found (idempotent delete)
                return {"status": "deleted"}

            db.delete(conversation)  # Messages cascade-deleted via FK
            db.commit()

            return {"status": "deleted"}
    except Exception as e:
        logger.error(f"Failed to delete session: {e}", exc_info=True)
        # Don't raise error for delete operations
        return {"status": "error", "detail": str(e)}


async def chat_stream_generator(
    query: str,
    session_id: str | None,
    scope: ScopeContext,
):
    """
    Generate SSE stream for chat responses.

    Converts between frontend's expected format and backend's ChatKit format.
    """
    try:
        # Determine conversation ID
        conversation_id = session_id

        # Create or get conversation
        with get_db_context() as db:
            if conversation_id:
                conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
                if not conversation:
                    # Create new conversation
                    conversation = Conversation(
                        id=conversation_id,
                        session_id=conversation_id,
                        title=query[:50],
                    )
                    db.add(conversation)
                    db.commit()
            else:
                # Create new conversation
                conversation_id = str(uuid.uuid4())
                conversation = Conversation(
                    id=conversation_id,
                    session_id=conversation_id,
                    title=query[:50],
                )
                db.add(conversation)
                db.commit()

            # Save user message
            user_msg = Message(
                id=str(uuid.uuid4()),
                conversation_id=conversation_id,
                role="user",
                content=query,
            )
            db.add(user_msg)
            db.commit()

        # Determine module filter from scope
        module_filter = None
        if scope.type == "module" and scope.module_id:
            module_filter = scope.module_id

        # Stream response using the agent
        assistant_response = ""
        citations = []

        async for event_type, data in get_agent_response_stream(
            user_message=query,
            conversation_id=conversation_id,
            module_filter=module_filter,
        ):
            if event_type == "token":
                assistant_response += data
                # Send content event to frontend
                yield f"data: {json.dumps({'type': 'content', 'text': data})}\n\n"

            elif event_type == "citation":
                citations.append(data)
                # Send citation event to frontend
                yield f"data: {json.dumps({'type': 'citation', 'citation': data})}\n\n"

            elif event_type == "error":
                yield f"data: {json.dumps({'type': 'error', 'error': data})}\n\n"
                return

        # Save assistant message
        with get_db_context() as db:
            assistant_msg = Message(
                id=str(uuid.uuid4()),
                conversation_id=conversation_id,
                role="assistant",
                content=assistant_response,
                citations=json.dumps(citations) if citations else None,
            )
            db.add(assistant_msg)

            # Update conversation timestamp
            conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
            if conversation:
                conversation.updated_at = datetime.utcnow()

            db.commit()

        # Send done event
        yield f"data: {json.dumps({'type': 'done'})}\n\n"

    except Exception as e:
        logger.error(f"Error in chat stream: {e}", exc_info=True)
        yield f"data: {json.dumps({'type': 'error', 'error': 'An error occurred'})}\n\n"


@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that matches frontend expectations.

    Accepts chat requests and returns SSE stream with responses.
    """
    try:
        if not request.query or not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        return EventSourceResponse(
            chat_stream_generator(
                query=request.query,
                session_id=request.session_id,
                scope=request.scope,
            ),
            media_type="text/event-stream",
        )

    except Exception as e:
        logger.error(f"Chat endpoint error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")
