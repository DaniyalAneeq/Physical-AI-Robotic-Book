# Implementation Plan: Integrated RAG Chatbot

**Branch**: `003-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-chatbot/spec.md`

## Summary

Implement an embedded RAG (Retrieval-Augmented Generation) chatbot within the Physical AI Humanoid Robotics textbook. The chatbot answers learner questions using only textbook content, with citations to source material. Key features include selective content scoping (page, selection, module), multi-turn conversations, and WCAG 2.1 AA accessibility compliance.

**Technical Approach**: OpenAI ChatKit SDK for chat orchestration, FastAPI backend, Qdrant Cloud for vector search, Neon Serverless Postgres for session storage, and React components embedded in Docusaurus.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: OpenAI ChatKit SDK, FastAPI 0.115+, Qdrant Client 1.10+, psycopg3
**Storage**: Qdrant Cloud (vectors), Neon Serverless Postgres (sessions)
**Testing**: pytest (backend), Jest + React Testing Library (frontend)
**Target Platform**: Web (Docusaurus static site with API backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <3s p95 query-to-response latency, 60 req/min rate limit
**Constraints**: Free tier limits (Qdrant 1GB, Neon 3GB), no PII storage
**Scale/Scope**: ~10K content chunks, single-tenant, expected <100 concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Technical Accuracy | PASS | All code examples tested, API refs match official docs |
| II. Pedagogical Clarity | PASS | Responses include citations for learning reinforcement |
| III. Consistency & Standards | PASS | File naming follows Docusaurus conventions |
| IV. Code Quality | PASS | PEP 8 for Python, ESLint for TypeScript, explicit versioning |
| V. Integration Coherence | PASS | Chatbot enhances all modules without modification |
| VI. Technology Currency | PASS | Using latest stable versions of all dependencies |
| VII. RAG Architecture | PASS | ChatKit + FastAPI + Qdrant + Neon as specified |
| VIII. RAG Security & Privacy | PASS | Parameterized queries, session encryption, no PII |
| IX. RAG Versioning | PASS | All deps pinned in requirements.txt/package.json |
| X. Artifact Sequence | PASS | Following spec → plan → tasks → implement flow |
| XI. Content Quality Gates | N/A | Chatbot feature, not textbook content |
| XII. RAG Quality Gates | PASS | 95% accuracy target, <3s latency, accessibility |

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-chatbot/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Entity schemas
├── quickstart.md        # Developer setup guide
├── contracts/           # API contracts (OpenAPI)
│   └── openapi.yaml
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py              # FastAPI app entry point
│   ├── config.py            # Environment configuration
│   ├── models/
│   │   ├── session.py       # Session entity
│   │   ├── query.py         # Query request/response
│   │   └── chunk.py         # Content chunk
│   ├── services/
│   │   ├── chatkit_server.py    # ChatKit server implementation
│   │   ├── embedding.py         # OpenAI embedding service
│   │   ├── retrieval.py         # Qdrant search service
│   │   └── session.py           # Session management
│   ├── api/
│   │   ├── routes.py        # API route definitions
│   │   └── middleware.py    # Rate limiting, CORS
│   └── scripts/
│       └── embed_content.py # Content embedding pipeline
├── tests/
│   ├── contract/
│   │   └── test_api.py
│   ├── integration/
│   │   └── test_chat_flow.py
│   └── unit/
│       ├── test_retrieval.py
│       └── test_session.py
├── requirements.txt
└── Dockerfile

AIdd-book/
├── src/
│   ├── components/
│   │   ├── ChatbotPanel/
│   │   │   ├── index.tsx
│   │   │   ├── ChatMessage.tsx
│   │   │   ├── ChatInput.tsx
│   │   │   ├── ScopeSelector.tsx
│   │   │   └── styles.module.css
│   │   └── ChatbotToggle/
│   │       ├── index.tsx
│   │       └── styles.module.css
│   ├── hooks/
│   │   ├── useChatbot.ts
│   │   └── useTextSelection.ts
│   ├── services/
│   │   └── chatApi.ts
│   └── theme/
│       └── DocItem/
│           └── Layout/
│               └── index.tsx  # Swizzled to include chatbot
├── static/
│   └── chatbot-icon.svg
└── docusaurus.config.ts   # Updated with chatbot plugin config
```

**Structure Decision**: Web application structure with separate `backend/` directory for FastAPI service and frontend components integrated into existing `AIdd-book/` Docusaurus project.

## Architecture Diagrams

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      User's Browser                              │
│  ┌─────────────────┐  ┌─────────────────────────────────────┐   │
│  │ Docusaurus Site │  │         ChatbotPanel Component      │   │
│  │  (Static HTML)  │  │  ┌─────────┐ ┌────────────────────┐│   │
│  │                 │←→│  │ Messages│ │   ScopeSelector    ││   │
│  │  Module 1       │  │  └─────────┘ │ [Page|Selection|   ││   │
│  │  Module 2       │  │  ┌─────────┐ │  Module|All]       ││   │
│  │  Module 3       │  │  │  Input  │ └────────────────────┘│   │
│  │  ...            │  │  └─────────┘                        │   │
│  └─────────────────┘  └─────────────────────────────────────┘   │
└───────────────────────────────┬─────────────────────────────────┘
                                │ HTTPS (SSE for streaming)
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                     FastAPI Backend                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ Rate Limiter │  │    CORS      │  │   ChatKit Server     │   │
│  │  Middleware  │  │  Middleware  │  │   (SSE Streaming)    │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
│                                              │                   │
│  ┌──────────────────────────────────────────┼──────────────────┐│
│  │                  Services Layer          │                  ││
│  │  ┌────────────┐  ┌────────────┐  ┌──────┴───────┐          ││
│  │  │ Embedding  │  │ Retrieval  │  │   Session    │          ││
│  │  │  Service   │  │  Service   │  │   Service    │          ││
│  │  └─────┬──────┘  └─────┬──────┘  └──────┬───────┘          ││
│  └────────┼───────────────┼────────────────┼───────────────────┘│
└───────────┼───────────────┼────────────────┼────────────────────┘
            │               │                │
            ▼               ▼                ▼
     ┌──────────┐    ┌──────────────┐  ┌────────────────┐
     │ OpenAI   │    │ Qdrant Cloud │  │ Neon Postgres  │
     │ API      │    │ (Vectors)    │  │ (Sessions)     │
     └──────────┘    └──────────────┘  └────────────────┘
```

### Query Flow Sequence

```
User                Browser           Backend           Qdrant       OpenAI
  │                    │                 │                │            │
  │ Type question      │                 │                │            │
  │───────────────────>│                 │                │            │
  │                    │ POST /api/chat  │                │            │
  │                    │ {query, scope}  │                │            │
  │                    │────────────────>│                │            │
  │                    │                 │ Embed query    │            │
  │                    │                 │───────────────────────────>│
  │                    │                 │<───────────────────────────│
  │                    │                 │ Vector search  │            │
  │                    │                 │ (with filter)  │            │
  │                    │                 │───────────────>│            │
  │                    │                 │<───────────────│            │
  │                    │                 │ Generate response          │
  │                    │                 │ (with context) │            │
  │                    │                 │───────────────────────────>│
  │                    │                 │<──────(SSE stream)─────────│
  │                    │<──(SSE stream)──│                │            │
  │<──────────────────│                 │                │            │
  │ See response       │                 │                │            │
  │ with citations     │                 │                │            │
```

### Embedding Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    Embedding Pipeline (Offline)                  │
└─────────────────────────────────────────────────────────────────┘

   AIdd-book/docs/**/*.md
          │
          ▼
   ┌──────────────────┐
   │  Markdown Parser │ ← Extract headings, paragraphs, code blocks
   └────────┬─────────┘
            │
            ▼
   ┌──────────────────┐
   │   Chunker        │ ← Split by section, max 512 tokens
   │   + Metadata     │ ← Attach: module, chapter, section, path
   └────────┬─────────┘
            │
            ▼
   ┌──────────────────┐
   │  OpenAI API      │ ← text-embedding-3-small (1536 dims)
   │  Embeddings      │
   └────────┬─────────┘
            │
            ▼
   ┌──────────────────┐
   │  Qdrant Upsert   │ ← Store vector + payload
   └──────────────────┘
```

### Session Management

```
┌─────────────────────────────────────────────────────────────────┐
│                    Session Lifecycle                             │
└─────────────────────────────────────────────────────────────────┘

  New User Visit          Active Session           Session Expired
       │                        │                        │
       ▼                        ▼                        ▼
  ┌──────────┐           ┌──────────────┐         ┌──────────────┐
  │ Generate │           │ Load Context │         │ Cleanup Job  │
  │ Session  │           │ from Postgres│         │ (24h inact.) │
  │ UUID     │           │              │         │              │
  └────┬─────┘           └──────┬───────┘         └──────┬───────┘
       │                        │                        │
       ▼                        ▼                        ▼
  ┌──────────┐           ┌──────────────┐         ┌──────────────┐
  │ Create   │           │ Append Query │         │ DELETE FROM  │
  │ Session  │           │ to History   │         │ sessions     │
  │ Row      │           │              │         │ WHERE ...    │
  └────┬─────┘           └──────┬───────┘         └──────────────┘
       │                        │
       ▼                        ▼
  ┌──────────┐           ┌──────────────┐
  │ Return   │           │ Update       │
  │ Session  │           │ last_activity│
  │ Cookie   │           │              │
  └──────────┘           └──────────────┘
```

## Complexity Tracking

> **No violations identified.** Architecture follows constitution principles.

| Aspect | Complexity | Justification |
|--------|------------|---------------|
| Separate backend service | Medium | Required for secure API key storage and rate limiting |
| Qdrant Cloud external service | Low | Simpler than self-hosted, free tier sufficient |
| ChatKit SDK | Low | Reduces custom code, official OpenAI support |

## Implementation Milestones

### Milestone 1: Backend Foundation (US1 Partial)
- FastAPI project setup with configuration
- Qdrant Cloud connection and health check
- Neon Postgres connection and session table
- Basic rate limiting middleware

### Milestone 2: Embedding Pipeline (US1 Partial)
- Markdown parser for textbook content
- Chunking logic with metadata extraction
- OpenAI embedding generation
- Qdrant upsert with payload

### Milestone 3: Chat API (US1 Complete)
- ChatKit server implementation
- Query embedding and retrieval
- Response generation with citations
- SSE streaming endpoint

### Milestone 4: Frontend Chat UI (US1 + US2)
- ChatbotPanel React component
- Message display with citations
- ScopeSelector (page/selection/module/all)
- Text selection hook for US2

### Milestone 5: Session & Multi-turn (US3)
- Session creation and persistence
- Conversation history management
- Context window handling
- Session expiration cleanup

### Milestone 6: Module Scoping (US4)
- Module dropdown in ScopeSelector
- Filter queries by module metadata
- Update scope on navigation

### Milestone 7: Polish & Quality Gates
- Accessibility audit (axe-core)
- Performance testing (<3s p95)
- Accuracy evaluation (95% target)
- Build validation (Docusaurus)

## Testing Strategy

### Unit Tests
- `test_retrieval.py`: Vector search with filters
- `test_session.py`: Session lifecycle, expiration
- `test_embedding.py`: Chunk generation, metadata

### Integration Tests
- `test_chat_flow.py`: End-to-end query processing
- `test_rate_limiting.py`: Rate limit enforcement

### Contract Tests
- `test_api.py`: OpenAPI schema validation
- Response format verification

### Accessibility Tests
- axe-core automated checks
- Keyboard navigation manual testing
- Screen reader testing (VoiceOver/NVDA)

### Performance Tests
- Latency benchmarks (100 sample queries)
- Concurrent request load testing

## Dependencies

### Backend (requirements.txt)
```
openai-chatkit>=0.1.0
openai>=1.50.0
fastapi>=0.115.0
uvicorn>=0.30.0
qdrant-client>=1.10.0
psycopg[binary]>=3.2.0
python-dotenv>=1.0.0
pydantic>=2.8.0
```

### Frontend (package.json additions)
```json
{
  "dependencies": {
    "@types/react": "^18.3.0"
  }
}
```

### Environment Variables
```
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgresql://...
```

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Qdrant free tier limits | Monitor usage, implement caching for frequent queries |
| OpenAI API latency | Stream responses, show loading state immediately |
| Session storage growth | Automatic cleanup job, conversation history limits |
| Build integration issues | Test Docusaurus build in CI before merge |
