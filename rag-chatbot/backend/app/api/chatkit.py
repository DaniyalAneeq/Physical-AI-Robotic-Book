"""ChatKit protocol handler for streaming conversations."""

import json
import logging
import time
import uuid
from datetime import datetime

from fastapi import APIRouter, HTTPException, Request
from sse_starlette.sse import EventSourceResponse

from app.models.conversation import Conversation
from app.models.database import get_db_context
from app.models.message import Message
from app.models.query_log import QueryLog
from app.services.agent import get_agent_response_stream

router = APIRouter()
logger = logging.getLogger(__name__)


async def chatkit_stream_generator(
    user_message: str,
    session_id: str,
    conversation_id: str | None = None,
    module_filter: str | None = None,
):
    """
    Generate SSE stream for ChatKit protocol.

    Args:
        user_message: User's input message
        session_id: Browser session UUID from localStorage
        conversation_id: Existing conversation ID (if continuing conversation)
        module_filter: Optional module filter for scoped queries

    Yields:
        SSE events in ChatKit protocol format
    """
    start_time = time.time()
    assistant_response = ""
    citations = []

    try:
        # Create or load conversation
        with get_db_context() as db:
            if conversation_id:
                conversation = (
                    db.query(Conversation).filter(Conversation.id == conversation_id).first()
                )
                if not conversation:
                    raise HTTPException(status_code=404, detail="Conversation not found")
            else:
                # Create new conversation
                conversation = Conversation(
                    id=str(uuid.uuid4()),
                    session_id=session_id,
                    title=user_message[:50],  # Use first 50 chars as title
                )
                db.add(conversation)
                db.commit()
                conversation_id = conversation.id

            # Save user message
            user_msg = Message(
                id=str(uuid.uuid4()),
                conversation_id=conversation_id,
                role="user",
                content=user_message,
            )
            db.add(user_msg)
            db.commit()

        # Stream agent response
        async for event_type, data in get_agent_response_stream(
            user_message=user_message,
            conversation_id=conversation_id,
            module_filter=module_filter,
        ):
            if event_type == "token":
                # Accumulate response
                assistant_response += data
                # Send token to client
                yield {
                    "event": "message",
                    "data": json.dumps({"type": "token", "content": data}),
                }

            elif event_type == "citation":
                citations.append(data)
                yield {
                    "event": "message",
                    "data": json.dumps({"type": "citation", "content": data}),
                }

            elif event_type == "error":
                yield {
                    "event": "error",
                    "data": json.dumps({"error": data}),
                }
                return

        # Save assistant message with citations
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

        # Log query for analytics
        latency_ms = int((time.time() - start_time) * 1000)
        with get_db_context() as db:
            query_log = QueryLog(
                id=str(uuid.uuid4()),
                query_text=user_message,
                response_summary=assistant_response[:200],
                latency_ms=latency_ms,
                chunks_retrieved=len(citations),
            )
            db.add(query_log)
            db.commit()

        # Send completion event
        yield {
            "event": "done",
            "data": json.dumps(
                {
                    "conversation_id": conversation_id,
                    "latency_ms": latency_ms,
                }
            ),
        }

    except Exception as e:
        logger.error(f"Error in ChatKit stream: {e}", exc_info=True)
        yield {
            "event": "error",
            "data": json.dumps(
                {"error": "An error occurred while processing your request. Please try again."}
            ),
        }


@router.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    ChatKit protocol endpoint for streaming chat responses.

    Accepts ChatKit protocol requests and returns SSE stream with agent responses.

    Request body example:
    {
        "message": "What is ROS 2?",
        "session_id": "uuid-from-browser",
        "conversation_id": "uuid-or-null",
        "context": {
            "module_filter": "module-1"  // optional
        }
    }
    """
    try:
        body = await request.json()
        user_message = body.get("message", "").strip()
        session_id = body.get("session_id")
        conversation_id = body.get("conversation_id")
        context = body.get("context", {})
        module_filter = context.get("module_filter")

        if not user_message:
            raise HTTPException(status_code=400, detail="Message cannot be empty")

        if not session_id:
            raise HTTPException(status_code=400, detail="Session ID required")

        return EventSourceResponse(
            chatkit_stream_generator(
                user_message=user_message,
                session_id=session_id,
                conversation_id=conversation_id,
                module_filter=module_filter,
            ),
            media_type="text/event-stream",
        )

    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON in request body")
    except Exception as e:
        logger.error(f"ChatKit endpoint error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")
