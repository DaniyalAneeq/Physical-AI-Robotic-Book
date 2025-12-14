# Research: RAG Chatbot Full Rebuild

**Feature**: 004-rag-chatbot-rebuild
**Date**: 2025-12-12
**Status**: Complete

---

## 1. OpenAI Agents SDK Architecture

### Decision: Use OpenAI Agents SDK with `Runner.run_streamed()` for agent execution

**Rationale**:
- The Agents SDK provides a clean abstraction for building AI agents with tools, handoffs, and sessions
- `Runner.run_streamed()` enables real-time streaming responses, critical for <1 second first-token latency
- Built-in session management via `SQLiteSession` or custom session implementations
- Native support for function tools via `@function_tool` decorator

**Alternatives Considered**:
- LangChain: More complex, heavier abstraction layer, not required for this use case
- Direct OpenAI Chat API: Would require manual tool orchestration and session management
- Assistant API (previous implementation): Being replaced per requirements

**Key Implementation Patterns**:
```python
from agents import Agent, Runner, function_tool

@function_tool
async def retrieve_textbook_content(ctx, query: str) -> str:
    """Retrieves relevant textbook chunks from Qdrant."""
    # Implementation here
    return results

agent = Agent(
    name="TextbookAssistant",
    instructions="You are a helpful assistant for the Physical AI textbook...",
    tools=[retrieve_textbook_content],
)

# Streaming execution
result = Runner.run_streamed(agent, user_input, session=session)
async for event in result.stream_events():
    # Handle streaming events
```

---

## 2. OpenAI ChatKit Integration

### Decision: Use ChatKit Python server with FastAPI + ChatKit React for frontend

**Rationale**:
- ChatKit provides batteries-included chat UI with streaming support
- `ChatKitServer` class handles protocol communication with frontend
- `stream_agent_response()` helper converts Agents SDK events to ChatKit events
- Native support for widgets, history, and theming

**Alternatives Considered**:
- Custom WebSocket implementation: More work, less polished UX
- Vercel AI SDK: Different ecosystem, less OpenAI-native
- Previous React chatbot: Being replaced per requirements

**Key Implementation Patterns**:
```python
# Backend (FastAPI + ChatKit Server)
from chatkit.server import ChatKitServer, StreamingResult
from chatkit.agents import stream_agent_response, AgentContext

class TextbookChatKitServer(ChatKitServer):
    async def respond(self, thread, input, context):
        agent_context = AgentContext(thread=thread, store=self.store)
        result = Runner.run_streamed(self.agent, input, context=agent_context)
        async for event in stream_agent_response(agent_context, result):
            yield event

@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    result = await server.process(await request.body(), {})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")
```

```typescript
// Frontend (React + ChatKit)
import { ChatKit, useChatKit } from '@openai/chatkit-react';

function TextbookChat() {
  const { control } = useChatKit({
    api: { url: '/api/chatkit', domainKey: 'textbook' },
    history: { enabled: true },
    startScreen: {
      greeting: 'Ask me anything about Physical AI & Robotics!',
      prompts: [
        { label: 'What is ROS 2?', prompt: 'What is ROS 2?' },
        { label: 'Explain URDF', prompt: 'What is URDF?' },
      ],
    },
  });
  return <ChatKit control={control} className="h-full w-full" />;
}
```

---

## 3. Qdrant Vector Store Strategy

### Decision: Use Qdrant Cloud with `text-embedding-3-small` (1536 dimensions)

**Rationale**:
- Qdrant Cloud provides managed vector database with free tier
- `text-embedding-3-small` offers excellent quality-to-cost ratio (per clarification)
- Python client supports both sync and async operations
- Payload filtering enables module-specific queries

**Alternatives Considered**:
- Pinecone: More expensive, similar capabilities
- Weaviate: More complex setup
- ChromaDB: Less production-ready
- `text-embedding-3-large`: Higher cost, marginal quality improvement for this use case

**Key Implementation Patterns**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

# Initialize client
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create collection
client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
)

# Upsert vectors with metadata
client.upsert(
    collection_name="textbook_chunks",
    points=[
        PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                "text": chunk_text,
                "module": "module-1",
                "chapter": "01-introduction",
                "section": "Getting Started",
                "file_path": "docs/module-1/01-introduction.md",
            }
        )
    ]
)

# Search with optional module filter
results = client.query_points(
    collection_name="textbook_chunks",
    query=query_embedding,
    query_filter=models.Filter(
        must=[models.FieldCondition(key="module", match=models.MatchValue(value="module-1"))]
    ) if module_filter else None,
    limit=5,
)
```

---

## 4. Neon PostgreSQL Database Strategy

### Decision: Use Neon Serverless PostgreSQL with SQLAlchemy + Alembic

**Rationale**:
- Neon provides serverless PostgreSQL with autoscaling
- SQLAlchemy ORM for type-safe database operations
- Alembic for schema migrations (fresh start per requirements)
- 7-day conversation retention policy (per clarification)

**Alternatives Considered**:
- Supabase: More features than needed, higher complexity
- PlanetScale: MySQL-based, not PostgreSQL
- Local SQLite: Not production-ready, no serverless scaling

**Key Implementation Patterns**:
```python
# database.py
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
import os

DATABASE_URL = os.getenv("DATABASE_URL")
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# models.py
from sqlalchemy import Column, String, DateTime, Text, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID
import uuid

class Conversation(Base):
    __tablename__ = "conversations"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), nullable=False, index=True)
    title = Column(String(255), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, onupdate=datetime.utcnow)

class Message(Base):
    __tablename__ = "messages"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id"))
    role = Column(String(50), nullable=False)  # user, assistant
    content = Column(Text, nullable=False)
    citations = Column(Text, nullable=True)  # JSON string
    created_at = Column(DateTime, default=datetime.utcnow)

class QueryLog(Base):
    __tablename__ = "query_logs"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_text = Column(Text, nullable=False)
    response_summary = Column(Text, nullable=True)
    latency_ms = Column(Integer, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
```

---

## 5. Document Chunking Strategy

### Decision: Heading-based chunking with overlap

**Rationale**:
- Markdown documents have natural heading boundaries
- Heading-based chunks preserve semantic coherence
- Overlap (100-200 chars) ensures context isn't lost at boundaries
- Maximum chunk size of 1500 tokens prevents context window issues

**Alternatives Considered**:
- Fixed-size chunking: Loses semantic boundaries
- Sentence-based: Too granular for technical content
- Page-based: Not applicable to markdown

**Key Implementation Patterns**:
```python
import re
from typing import List, Dict

def chunk_markdown(content: str, file_path: str) -> List[Dict]:
    """Chunk markdown by headings with metadata extraction."""
    chunks = []

    # Split by headings (##, ###, ####)
    sections = re.split(r'(?=^#{2,4}\s)', content, flags=re.MULTILINE)

    for section in sections:
        if not section.strip():
            continue

        # Extract heading
        heading_match = re.match(r'^(#{2,4})\s+(.+)$', section, re.MULTILINE)
        heading = heading_match.group(2) if heading_match else "Introduction"

        # Extract module/chapter from file path
        path_parts = file_path.split('/')
        module = next((p for p in path_parts if p.startswith('module-')), None)
        chapter = path_parts[-1].replace('.md', '')

        chunks.append({
            "text": section.strip(),
            "module": module,
            "chapter": chapter,
            "section": heading,
            "file_path": file_path,
        })

    return chunks
```

---

## 6. UV Package Manager Setup

### Decision: Use UV for Python dependency management

**Rationale**:
- UV is significantly faster than pip/poetry
- Clean project initialization with `uv init`
- Lock file ensures reproducibility
- Native virtual environment management

**Alternatives Considered**:
- pip + venv: Slower, less reproducible
- Poetry: More complex, slower than UV
- Conda: Overkill for this project

**Key Commands**:
```bash
# Initialize new project
uv init backend
cd backend

# Add dependencies
uv add fastapi uvicorn openai-agents chatkit-python
uv add qdrant-client sqlalchemy alembic psycopg2-binary
uv add python-dotenv pydantic

# Development dependencies
uv add --dev pytest pytest-asyncio httpx ruff

# Run server
uv run uvicorn app.main:app --reload --port 8000
```

---

## 7. Error Handling & Fallback Strategy

### Decision: Circuit breaker pattern with graceful degradation

**Rationale**:
- External services (OpenAI, Qdrant, Neon) can fail
- Circuit breaker prevents cascade failures
- Graceful degradation maintains UX during outages

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential

class CircuitBreaker:
    def __init__(self, failure_threshold=5, reset_timeout=60):
        self.failures = 0
        self.threshold = failure_threshold
        self.last_failure = None
        self.reset_timeout = reset_timeout

    def is_open(self) -> bool:
        if self.failures >= self.threshold:
            if time.time() - self.last_failure > self.reset_timeout:
                self.failures = 0
                return False
            return True
        return False

@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
async def query_qdrant_with_retry(client, query_vector, **kwargs):
    return client.query_points(**kwargs)
```

---

## 8. Frontend Integration with Docusaurus

### Decision: Embed ChatKit as React component in Docusaurus

**Rationale**:
- Docusaurus supports React components natively
- ChatKit provides complete chat UI out of the box
- Can be embedded on any page or as floating widget

**Implementation**:
```typescript
// src/components/TextbookChat/index.tsx
import React from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';

export default function TextbookChat(): JSX.Element {
  const { control } = useChatKit({
    api: {
      url: process.env.CHATKIT_API_URL || 'http://localhost:8000/chatkit',
      domainKey: 'textbook-chat',
    },
    theme: { colorScheme: 'light', radius: 'round' },
    history: { enabled: true, showDelete: true },
  });

  return (
    <div className="textbook-chat-container">
      <ChatKit control={control} />
    </div>
  );
}
```

---

## Summary of Technical Decisions

| Area | Decision | Key Rationale |
|------|----------|---------------|
| Agent Framework | OpenAI Agents SDK | Native streaming, tool support, sessions |
| Frontend/Backend Protocol | OpenAI ChatKit | Batteries-included, streaming, widgets |
| Vector Database | Qdrant Cloud | Managed, free tier, payload filtering |
| Embedding Model | text-embedding-3-small | Cost-effective, 1536 dims |
| Relational DB | Neon PostgreSQL | Serverless, autoscaling |
| ORM | SQLAlchemy + Alembic | Type-safe, migrations |
| Package Manager | UV | Fast, reproducible |
| Chunking | Heading-based + overlap | Semantic coherence |
| Error Handling | Circuit breaker + retry | Graceful degradation |
| Session Strategy | Anonymous localStorage UUID | No auth per clarification |
| Retention | 7 days | Minimal storage per clarification |

---

## Remaining Questions (None)

All technical questions resolved through Context7 documentation review and user clarifications.
