# Implementation Plan: RAG Chatbot Full Rebuild

**Branch**: `004-rag-chatbot-rebuild` | **Date**: 2025-12-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-rag-chatbot-rebuild/spec.md`

## Summary

Complete rebuild of the RAG chatbot backend using OpenAI Agents SDK with `Runner.run_streamed()` for streaming responses, OpenAI ChatKit for frontend/backend protocol, Qdrant Cloud for vector storage with `text-embedding-3-small` embeddings (1536 dimensions), and Neon Serverless PostgreSQL for conversation persistence. Anonymous sessions via localStorage UUID with 7-day retention.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend/Docusaurus)
**Primary Dependencies**: FastAPI 0.115+, openai-agents, chatkit-python, qdrant-client 1.10+, SQLAlchemy 2.x, Alembic
**Storage**: Neon Serverless PostgreSQL (conversations, messages, query logs), Qdrant Cloud (vector embeddings)
**Testing**: pytest, pytest-asyncio, httpx (backend); Jest (frontend)
**Target Platform**: Linux server (Docker), Vercel/Netlify (frontend)
**Project Type**: Web application (backend + frontend integration)
**Performance Goals**: <2s p95 response latency, <1s first token streaming, 50 concurrent users
**Constraints**: Free tier limits for Qdrant Cloud (1GB) and Neon, 7-day conversation retention
**Scale/Scope**: ~100 markdown files indexed, ~500-1000 chunks, anonymous browser sessions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Max 3 production dependencies per layer | PASS | Backend: FastAPI, openai-agents, chatkit-python |
| No custom auth (use existing) | PASS | Anonymous sessions, no auth required |
| Repository pattern only if justified | PASS | Direct SQLAlchemy ORM, no repository pattern |
| Test coverage for critical paths | PENDING | To be verified during implementation |
| Circuit breaker for external services | PASS | Documented in research.md |

## Project Structure

### Documentation (this feature)

```text
specs/004-rag-chatbot-rebuild/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature requirements
├── research.md          # Phase 0 output - technical decisions
├── data-model.md        # Phase 1 output - entity definitions
├── quickstart.md        # Phase 1 output - developer setup
├── contracts/           # Phase 1 output - API contracts
│   └── openapi.yaml     # OpenAPI 3.1.0 specification
├── checklists/          # Quality validation
│   └── requirements.md  # Requirements checklist
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
rag-chatbot/
├── backend/
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py           # FastAPI app + ChatKit endpoint
│   │   ├── config.py         # Environment configuration
│   │   ├── api/
│   │   │   ├── __init__.py
│   │   │   ├── chatkit.py    # ChatKit protocol handler
│   │   │   ├── conversations.py  # Conversation CRUD
│   │   │   ├── health.py     # Health check endpoint
│   │   │   └── index.py      # Index management endpoints
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   ├── database.py   # SQLAlchemy engine setup
│   │   │   ├── conversation.py  # Conversation model
│   │   │   ├── message.py    # Message model
│   │   │   └── query_log.py  # QueryLog model
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── agent.py      # OpenAI Agents SDK setup
│   │   │   ├── embeddings.py # Embedding service
│   │   │   ├── vector_store.py  # Qdrant operations
│   │   │   └── chunker.py    # Markdown chunking
│   │   └── tools/
│   │       ├── __init__.py
│   │       └── retriever.py  # @function_tool for RAG retrieval
│   ├── alembic/
│   │   ├── env.py
│   │   └── versions/
│   ├── scripts/
│   │   ├── init_qdrant.py    # Collection initialization
│   │   └── index_textbook.py # Content indexing
│   ├── tests/
│   │   ├── conftest.py
│   │   ├── test_agent.py
│   │   ├── test_embeddings.py
│   │   └── test_api.py
│   ├── pyproject.toml
│   ├── uv.lock
│   └── .env.example
│
└── frontend/                  # Docusaurus integration
    └── src/
        └── components/
            └── TextbookChat/
                ├── index.tsx
                └── styles.module.css

AIdd-book/                     # Existing Docusaurus site
└── src/
    └── components/
        └── TextbookChat/      # ChatKit widget component
```

**Structure Decision**: Web application structure with separate `backend/` directory under `rag-chatbot/`. Frontend component integrated into existing `AIdd-book/` Docusaurus site.

## Key Technical Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Agent Framework | OpenAI Agents SDK | Native streaming via `Runner.run_streamed()`, `@function_tool` decorator |
| Frontend Protocol | OpenAI ChatKit | Batteries-included streaming UI, SSE support |
| Vector Database | Qdrant Cloud | Managed, free tier, payload filtering |
| Embedding Model | text-embedding-3-small | 1536 dimensions, cost-effective |
| Relational DB | Neon PostgreSQL | Serverless, autoscaling |
| ORM | SQLAlchemy + Alembic | Type-safe, migrations |
| Package Manager | UV | Fast, reproducible |
| Chunking Strategy | Heading-based + overlap | Semantic coherence |
| Session Strategy | Anonymous localStorage UUID | No auth per requirements |
| Retention Policy | 7 days | Minimal storage |

## Implementation Phases

### Phase 0: Research (Complete)
- [x] Context7 documentation review (Agents SDK, ChatKit, Qdrant, Neon)
- [x] Technical decisions documented in research.md
- [x] All NEEDS CLARIFICATION items resolved

### Phase 1: Design (Complete)
- [x] Data model defined in data-model.md
- [x] API contracts in contracts/openapi.yaml
- [x] Developer quickstart in quickstart.md

### Phase 2: Implementation (Next - via /sp.tasks)
- [ ] Backend skeleton with FastAPI
- [ ] Database models and migrations
- [ ] Qdrant integration and chunking
- [ ] Agent setup with retriever tool
- [ ] ChatKit endpoint integration
- [ ] Frontend ChatKit widget
- [ ] End-to-end testing

## Artifacts Generated

| Artifact | Path | Description |
|----------|------|-------------|
| Research | [research.md](./research.md) | 8 technical decisions with code patterns |
| Data Model | [data-model.md](./data-model.md) | PostgreSQL + Qdrant entity definitions |
| API Contract | [contracts/openapi.yaml](./contracts/openapi.yaml) | OpenAPI 3.1.0 specification |
| Quickstart | [quickstart.md](./quickstart.md) | Developer setup instructions |

## Complexity Tracking

No violations identified. Project adheres to constitution constraints:
- 3 primary backend dependencies (FastAPI, openai-agents, chatkit-python)
- Direct SQLAlchemy ORM without repository abstraction
- Anonymous sessions without custom auth implementation

## Next Steps

1. Run `/sp.tasks` to generate implementation task breakdown
2. Execute tasks in dependency order
3. Validate against success criteria from spec.md

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Qdrant free tier limits | Medium | Medium | Monitor usage, implement pagination |
| OpenAI rate limits | Low | High | Implement circuit breaker, retry logic |
| Cold start latency (Neon) | Medium | Low | Connection pooling, warm-up requests |
