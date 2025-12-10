"""API route definitions."""

import json
import time
from typing import Optional
from uuid import UUID

from fastapi import APIRouter, HTTPException, Request, status
from fastapi.responses import StreamingResponse, Response, JSONResponse
from chatkit.server import StreamingResult, NonStreamingResult

from ..config import get_settings
from ..models.query import ChatRequest, ChatResponse, Error, HealthStatus
from ..models.session import Session, SessionCreate, SessionResponse, ScopeContext
from ..services.chatkit_server import chatkit_server
from ..services.embedding import embedding_service
from ..services.logging import logging_service
from ..services.retrieval import retrieval_service
from ..services.session import session_service

settings = get_settings()

router = APIRouter()


@router.get("/health", response_model=HealthStatus)
async def health_check():
    """Health check endpoint.

    Returns the health status of the API and its dependencies.
    """
    qdrant_status = "connected" if await retrieval_service.validate_connection() else "disconnected"
    postgres_status = "connected" if await session_service.validate_connection() else "disconnected"
    openai_status = "connected" if await embedding_service.validate_connection() else "disconnected"

    status_value = "healthy"
    if "disconnected" in [qdrant_status, postgres_status, openai_status]:
        status_value = "unhealthy"

    return HealthStatus(
        status=status_value,
        version="1.0.0",
        dependencies={
            "qdrant": qdrant_status,
            "postgres": postgres_status,
            "openai": openai_status,
        },
    )


@router.post("/chatkit")
async def chatkit_endpoint(request: Request) -> Response:
    """Process ChatKit requests.

    This is the main ChatKit endpoint that handles all ChatKit protocol requests.
    It delegates request handling to the ChatKit server which manages agent execution,
    tool invocation, and event streaming.
    """
    payload = await request.body()
    context = {"request": request}

    result = await chatkit_server.process(payload, context)

    if isinstance(result, StreamingResult):
        return StreamingResponse(
            result,
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    if isinstance(result, NonStreamingResult):
        return Response(content=result.json, media_type="application/json")

    # Fallback for unexpected result types
    return JSONResponse(content={"error": "Unexpected result type"}, status_code=500)


@router.post("/chat")
async def process_chat(request: ChatRequest):
    """Process a chat query with SSE streaming response (legacy endpoint).

    Accepts a user question with optional scope constraints, retrieves relevant
    textbook content, and streams a response with citations via SSE.

    Note: This is maintained for backward compatibility. New integrations should
    use the /chatkit endpoint with the ChatKit React component.
    """
    start_time = time.time()

    # Get or create session
    session = None
    if request.session_id:
        session = await session_service.get(request.session_id)
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=Error(
                    error="not_found",
                    message="Session not found or expired",
                ).model_dump(),
            )

    async def event_generator():
        """Generate SSE events using legacy format."""
        from ..models.query import StreamEvent, Citation
        from ..models.chunk import ChunkPayload
        from ..models.session import ScopeType

        response_text = ""
        citations = []
        chunk_count = 0

        try:
            # Get embedding for query
            query_vector = await embedding_service.embed(request.query)

            # Retrieve relevant chunks
            chunks = await retrieval_service.search(
                query_vector=query_vector,
                scope=request.scope,
                limit=settings.max_chunks_retrieved,
            )

            # Build context
            context_parts = []
            for i, (chunk, score) in enumerate(chunks, 1):
                context_parts.append(
                    f"[Source {i}] (Chapter: {chunk.chapter_title}, Section: {chunk.section_title})\n"
                    f"{chunk.text}"
                )

            context = "\n\n---\n\n".join(context_parts)

            system_prompt = f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Your role is to answer learner questions using ONLY the provided textbook content.

IMPORTANT RULES:
1. Only answer questions based on the provided context from the textbook
2. If the question cannot be answered from the provided context, say "I can only answer questions about the textbook content."
3. Always cite your sources by referring to the chapter and section
4. Be concise but thorough
5. Use markdown formatting for code blocks

TEXTBOOK CONTEXT:
{context}"""

            # Stream from OpenAI
            from openai import AsyncOpenAI
            client = AsyncOpenAI(api_key=settings.openai_api_key)

            stream = await client.chat.completions.create(
                model=settings.chat_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": request.query}
                ],
                stream=True,
                max_tokens=1000,
                temperature=0.7,
            )

            async for chunk in stream:
                if chunk.choices and chunk.choices[0].delta.content:
                    text = chunk.choices[0].delta.content
                    response_text += text
                    event = StreamEvent(type="content", text=text)
                    yield f"data: {json.dumps(event.model_dump(exclude_none=True))}\n\n"

            # Yield citations
            for chunk_payload, score in chunks:
                text = chunk_payload.text or ""
                if len(text) > 180:
                    excerpt = text[:177].rsplit(" ", 1)[0] + "..."
                else:
                    excerpt = text[:180] if text else "No excerpt available"

                citation = Citation(
                    chunk_id=f"{chunk_payload.chapter_id}-{chunk_payload.section_id}-{chunk_payload.paragraph_id}",
                    chapter_title=(chunk_payload.chapter_title or "Unknown").strip("'\""),
                    section_title=(chunk_payload.section_title or "Unknown").strip("'\""),
                    paragraph_number=chunk_payload.paragraph_id,
                    relevance_score=round(score, 2),
                    excerpt=excerpt,
                )
                chunk_count += 1
                event = StreamEvent(type="citation", citation=citation)
                yield f"data: {json.dumps(event.model_dump(exclude_none=True))}\n\n"

            yield f"data: {json.dumps({'type': 'done'})}\n\n"

        except Exception as e:
            event = StreamEvent(type="error", error=str(e))
            yield f"data: {json.dumps(event.model_dump(exclude_none=True))}\n\n"

        # Log query (after completion)
        latency_ms = int((time.time() - start_time) * 1000)
        try:
            await logging_service.log_query(
                session_id=request.session_id,
                query_text=request.query,
                scope_type=request.scope.type.value,
                response_latency_ms=latency_ms,
                chunk_count=chunk_count,
            )
        except Exception:
            pass

        # Update session
        if session and response_text:
            try:
                await session_service.update(
                    session_id=session.id,
                    query=request.query,
                    response=response_text,
                    scope_context=request.scope,
                )
            except Exception:
                pass

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@router.post("/sessions", response_model=SessionResponse, status_code=status.HTTP_201_CREATED)
async def create_session(request: Optional[SessionCreate] = None):
    """Create a new session.

    Creates a new conversation session and returns the session ID.
    """
    scope_context = request.scope_context if request else None
    session = await session_service.create(scope_context)
    return SessionResponse.from_session(session)


@router.get("/sessions/{session_id}", response_model=SessionResponse)
async def get_session(session_id: UUID):
    """Get session details.

    Retrieves the current state of a session including conversation history.
    """
    session = await session_service.get(session_id)
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=Error(
                error="not_found",
                message="Session not found or expired",
            ).model_dump(),
        )
    return SessionResponse.from_session(session)


@router.delete("/sessions/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_session(session_id: UUID):
    """End a session.

    Explicitly ends and deletes a session.
    """
    deleted = await session_service.delete(session_id)
    if not deleted:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=Error(
                error="not_found",
                message="Session not found or expired",
            ).model_dump(),
        )
