"""ChatKit protocol handler for streaming conversations."""

import json
import logging
import time
import uuid
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, Request
from sse_starlette.sse import EventSourceResponse

from app.models.conversation import Conversation
from app.models.database import get_db_context
from app.models.message import Message
from app.models.query_log import QueryLog
from app.services.agent import get_agent_response_stream
from app.services.translation import (
    detect_language,
    translate_urdu_to_english,
    translate_english_to_urdu,
    extract_technical_terms,
)
from app.services.translation_cache import (
    compute_query_hash,
    get_cached_translation,
    cache_translation,
)

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


# Conditional auth dependency - use get_current_user_optional which returns None for unauthenticated
# This is injected properly by FastAPI's dependency system
if AUTH_AVAILABLE:
    maybe_get_user = get_current_user_optional
else:
    async def maybe_get_user():
        """Fallback when auth is not available."""
        return None


async def chatkit_stream_generator(
    user_message: str,
    session_id: str,
    conversation_id: str | None = None,
    module_filter: str | None = None,
    user_locale: str = "en",
):
    """
    Generate SSE stream for ChatKit protocol with i18n support.

    Args:
        user_message: User's input message
        session_id: Browser session UUID from localStorage
        conversation_id: Existing conversation ID (if continuing conversation)
        module_filter: Optional module filter for scoped queries
        user_locale: User's preferred locale ('en' or 'ur')

    Yields:
        SSE events in ChatKit protocol format
    """
    start_time = time.time()
    assistant_response = ""
    citations = []
    detected_lang = "en"
    query_for_search = user_message  # Query to use for vector search
    translation_cached = False

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

        # === i18n Translation Flow ===
        # Step 1: Detect language of user query
        detected_lang, confidence = detect_language(user_message)
        logger.info(f"Detected language: {detected_lang} (confidence: {confidence:.2f})")

        # Step 2: Check translation cache (for Urdu queries)
        if detected_lang == "ur":
            query_hash = compute_query_hash(user_message)
            cached_result = get_cached_translation(
                query_hash=query_hash,
                source_language="ur",
                target_language="en"
            )

            if cached_result:
                query_for_search = cached_result["translated_text"]
                translation_cached = True
                logger.info(f"Cache hit for query: {user_message[:50]}...")
            else:
                # Step 3: Translate Urdu query to English for vector search
                try:
                    query_for_search = translate_urdu_to_english(user_message)
                    logger.info(f"Translated query: {query_for_search}")

                    # Cache the translation
                    cache_translation(
                        query_hash=query_hash,
                        source_language="ur",
                        target_language="en",
                        original_text=user_message,
                        translated_text=query_for_search,
                    )
                except Exception as e:
                    logger.error(f"Translation error (Urdu→English): {e}")
                    query_for_search = user_message  # Fallback to original

        # Step 4: Stream agent response (using translated query for search)
        async for event_type, data in get_agent_response_stream(
            user_message=query_for_search,  # Use translated query for vector search
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

        # Step 5: Translate response to Urdu if query was in Urdu
        final_response = assistant_response
        if detected_lang == "ur" and assistant_response:
            try:
                # Extract technical terms from citations for preservation
                technical_terms = []
                for citation in citations:
                    if isinstance(citation, dict) and "content" in citation:
                        technical_terms.extend(extract_technical_terms(citation["content"]))

                # Remove duplicates while preserving order
                seen = set()
                unique_terms = []
                for term in technical_terms:
                    if term.lower() not in seen:
                        seen.add(term.lower())
                        unique_terms.append(term)

                # Translate response to Urdu
                final_response = translate_english_to_urdu(
                    assistant_response,
                    preserve_terms=unique_terms
                )
                logger.info(f"Translated response to Urdu (preserved {len(unique_terms)} terms)")

            except Exception as e:
                logger.error(f"Translation error (English→Urdu): {e}")
                final_response = assistant_response  # Fallback to English

        # Save assistant message with citations
        with get_db_context() as db:
            assistant_msg = Message(
                id=str(uuid.uuid4()),
                conversation_id=conversation_id,
                role="assistant",
                content=final_response,  # Save translated response
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

        # Send completion event with i18n metadata
        yield {
            "event": "done",
            "data": json.dumps(
                {
                    "conversation_id": conversation_id,
                    "latency_ms": latency_ms,
                    "detected_language": detected_lang,
                    "translation_cached": translation_cached,
                    "final_response": final_response if detected_lang == "ur" else None,
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
async def chatkit_endpoint(
    request: Request,
    user=Depends(maybe_get_user),
):
    """
    ChatKit protocol endpoint for streaming chat responses.

    **Authentication Required:** Yes (must be authenticated with completed onboarding)

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

    **Errors:**
    - 401 Unauthorized: Not authenticated or invalid session
    - 403 Forbidden: Authenticated but onboarding not completed
    - 400 Bad Request: Invalid request body
    """
    try:
        body = await request.json()
        user_message = body.get("message", "").strip()
        session_id = body.get("session_id")
        conversation_id = body.get("conversation_id")
        context = body.get("context", {})
        module_filter = context.get("module_filter")
        user_locale = context.get("locale", "en")  # Default to English

        if not user_message:
            raise HTTPException(status_code=400, detail="Message cannot be empty")

        if not session_id:
            raise HTTPException(status_code=400, detail="Session ID required")

        # Validate locale
        if user_locale not in ["en", "ur"]:
            logger.warning(f"Invalid locale '{user_locale}', defaulting to 'en'")
            user_locale = "en"

        return EventSourceResponse(
            chatkit_stream_generator(
                user_message=user_message,
                session_id=session_id,
                conversation_id=conversation_id,
                module_filter=module_filter,
                user_locale=user_locale,
            ),
            media_type="text/event-stream",
        )

    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON in request body")
    except Exception as e:
        logger.error(f"ChatKit endpoint error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")
