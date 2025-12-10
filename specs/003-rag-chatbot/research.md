# Research: Integrated RAG Chatbot

**Feature Branch**: `003-rag-chatbot`
**Date**: 2025-12-10
**Status**: Complete

## Technology Decisions

### 1. OpenAI ChatKit SDK Integration

**Decision**: Use OpenAI ChatKit Python SDK for backend chatbot logic and streaming responses.

**Rationale**:
- ChatKit provides a high-level abstraction for building chat applications with OpenAI models
- Built-in support for streaming responses via Server-Sent Events (SSE)
- Native integration with OpenAI Agents SDK for tool calling and structured outputs
- Thread management abstraction handles conversation context automatically
- Reduces boilerplate for session management and message formatting

**Alternatives Considered**:
- **Raw OpenAI Python SDK**: More control but requires manual implementation of streaming, thread management, and message formatting. Rejected due to increased development effort.
- **LangChain**: Feature-rich but heavier dependency footprint. Rejected for simplicity in this use case.

**Implementation Pattern**:
```python
class TextbookChatKitServer(ChatKitServer):
    def __init__(self, data_store: Store, attachment_store: AttachmentStore | None = None):
        super().__init__(data_store, attachment_store)

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | None,
        context: Any,
    ) -> AsyncIterator[ThreadStreamEvent]:
        # Retrieve relevant content from Qdrant
        # Generate response with citations
        # Stream via ThreadStreamEvent
```

### 2. FastAPI Backend Architecture

**Decision**: FastAPI with async endpoints for all chatbot operations.

**Rationale**:
- Native async/await support for non-blocking I/O with Qdrant and OpenAI
- Automatic OpenAPI documentation generation
- Built-in dependency injection for database sessions
- High performance (comparable to Node.js and Go)
- Excellent Python ecosystem compatibility

**Key Patterns**:
- Use `APIRouter` for modular endpoint organization
- Implement session dependency with `Depends()` for database access
- Use `StreamingResponse` for SSE streaming from ChatKit

**Endpoints Required**:
| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/chat` | POST | Process chat query, return streaming response |
| `/api/sessions` | POST | Create new session |
| `/api/sessions/{id}` | GET | Retrieve session context |
| `/api/sessions/{id}` | DELETE | End session |
| `/api/health` | GET | Health check for deployment |

### 3. Qdrant Vector Store Configuration

**Decision**: Qdrant Cloud Free Tier with cosine similarity and payload filtering.

**Rationale**:
- Free tier provides 1GB storage (sufficient for textbook corpus ~10K chunks)
- Cloud-hosted eliminates infrastructure management
- Payload filtering enables scope-based retrieval (chapter, module, selected text)
- Native Python client with async support

**Schema Design**:
```python
collection_config = {
    "collection_name": "textbook_content",
    "vectors_config": {
        "size": 1536,  # OpenAI text-embedding-3-small
        "distance": "Cosine"
    }
}

payload_schema = {
    "module_id": "string",      # e.g., "module-1"
    "chapter_id": "string",     # e.g., "chapter-3"
    "section_id": "string",     # e.g., "section-3.2"
    "paragraph_id": "integer",  # e.g., 4
    "content_type": "string",   # "paragraph" | "code_block" | "heading"
    "text": "string",           # Original text for citation
    "file_path": "string"       # Source markdown file
}
```

**Query Pattern**:
```python
hits = client.query_points(
    collection_name="textbook_content",
    query=query_embedding,
    query_filter=Filter(
        must=[
            FieldCondition(key="chapter_id", match=MatchValue(value="chapter-3"))
        ]
    ),
    limit=5
)
```

### 4. Neon Serverless Postgres for Session Storage

**Decision**: Neon Serverless Postgres for session metadata and analytics logging.

**Rationale**:
- Serverless model scales to zero when not in use (cost-effective)
- PostgreSQL compatibility for reliable session management
- Connection pooling via Neon proxy handles concurrent requests
- Free tier provides 3GB storage (ample for session data)

**Schema Design**:
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    conversation_history JSONB DEFAULT '[]',
    scope_context JSONB DEFAULT '{}'
);

CREATE TABLE query_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES sessions(id),
    query_text TEXT NOT NULL,
    scope_type VARCHAR(50),  -- 'page' | 'selection' | 'module' | 'all'
    response_latency_ms INTEGER,
    chunk_count INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Index for session expiration cleanup
CREATE INDEX idx_sessions_last_activity ON sessions(last_activity);

-- Index for analytics queries
CREATE INDEX idx_query_logs_created_at ON query_logs(created_at);
```

### 5. Embedding Pipeline

**Decision**: OpenAI `text-embedding-3-small` for content embedding.

**Rationale**:
- 1536 dimensions provides good accuracy/cost balance
- Native integration with OpenAI ecosystem (consistent with ChatKit)
- Lower cost than `text-embedding-3-large` ($0.02/1M tokens vs $0.13/1M)
- Sufficient quality for textbook content retrieval

**Chunking Strategy**:
- Split markdown by headers (h1, h2, h3) to preserve section boundaries
- Further split paragraphs exceeding 512 tokens
- Preserve code blocks as atomic units
- Include metadata (file path, section hierarchy) in payload

**Pipeline Steps**:
1. Parse markdown files from `AIdd-book/docs/`
2. Extract content chunks with metadata
3. Generate embeddings via OpenAI API
4. Upsert to Qdrant with payload

### 6. Frontend Integration in Docusaurus

**Decision**: Custom React component embedded in Docusaurus theme.

**Rationale**:
- Docusaurus uses React, enabling seamless component integration
- Can access page context (current route) for automatic scoping
- CSS Modules for isolated styling
- Built-in SSR compatibility

**Implementation Approach**:
- Create `ChatbotPanel` component in `src/components/`
- Swizzle `DocItem` layout to inject chatbot toggle
- Use `useLocation()` hook for current page detection
- Implement text selection API for highlighted content scope

**Accessibility Requirements**:
- ARIA labels for all interactive elements
- Focus management for modal/panel
- Keyboard navigation (Tab, Enter, Escape)
- Screen reader announcements for new messages

### 7. Rate Limiting Strategy

**Decision**: Token bucket algorithm with 60 requests/minute per session.

**Rationale**:
- Prevents abuse while allowing burst traffic
- Session-based (not IP-based) for consistent experience across networks
- Aligned with constitution requirement (FR-007)

**Implementation**:
```python
from fastapi import Request, HTTPException
from datetime import datetime, timedelta

class RateLimiter:
    def __init__(self, max_requests: int = 60, window_seconds: int = 60):
        self.max_requests = max_requests
        self.window = timedelta(seconds=window_seconds)
        self.requests: dict[str, list[datetime]] = {}

    async def check(self, session_id: str) -> bool:
        now = datetime.utcnow()
        if session_id not in self.requests:
            self.requests[session_id] = []

        # Remove expired requests
        self.requests[session_id] = [
            t for t in self.requests[session_id]
            if now - t < self.window
        ]

        if len(self.requests[session_id]) >= self.max_requests:
            return False

        self.requests[session_id].append(now)
        return True
```

### 8. Privacy & Logging

**Decision**: Anonymized logging with no PII storage.

**Rationale**:
- Constitution requires privacy compliance (VIII. RAG Security & Privacy)
- Query patterns provide analytics value without identifying users
- Session IDs are randomly generated UUIDs (not linked to user identity)

**Implementation**:
- Log: query text, scope type, latency, chunk count, timestamp
- Do NOT log: IP addresses, user agents, cookies
- Session cleanup: Automatic deletion after 24 hours of inactivity

## Dependency Versions

| Package | Version | Purpose |
|---------|---------|---------|
| `openai-chatkit` | `^0.1.0` | ChatKit server SDK |
| `openai` | `^1.50.0` | Embeddings and completions |
| `fastapi` | `^0.115.0` | Web framework |
| `uvicorn` | `^0.30.0` | ASGI server |
| `qdrant-client` | `^1.10.0` | Vector database client |
| `psycopg[binary]` | `^3.2.0` | PostgreSQL driver (async) |
| `python-dotenv` | `^1.0.0` | Environment configuration |
| `pydantic` | `^2.8.0` | Data validation |

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| How to handle text selection across code blocks? | Treat selected text as search query, match against Qdrant |
| Session storage: Redis vs Postgres? | Postgres - simpler stack, sufficient for expected load |
| Embedding model size? | `text-embedding-3-small` - cost/quality balance |
| Frontend framework for chatbot? | Native React component in Docusaurus |
