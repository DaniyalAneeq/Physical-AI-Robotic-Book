# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/003-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/openapi.yaml, research.md, quickstart.md

**Tests**: Tests are included as they support quality gates defined in the constitution (95% accuracy, <3s latency, accessibility).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `AIdd-book/src/`
- Paths are relative to repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure per plan.md: backend/src/{models,services,api,scripts}/, backend/tests/{unit,integration,contract}/
- [X] T002 [P] Create backend/requirements.txt with pinned dependencies: openai-chatkit>=0.1.0, openai>=1.50.0, fastapi>=0.115.0, uvicorn>=0.30.0, qdrant-client>=1.10.0, psycopg[binary]>=3.2.0, python-dotenv>=1.0.0, pydantic>=2.8.0, pytest>=8.0.0
- [X] T003 [P] Create backend/.env.example with placeholder environment variables: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, HOST, PORT, DEBUG
- [X] T004 [P] Create backend/src/config.py with Pydantic Settings for environment configuration
- [X] T005 Create backend/src/main.py with FastAPI app initialization, CORS middleware, and health check endpoint
- [X] T006 [P] Create backend/Dockerfile for containerized deployment

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Create backend/src/models/chunk.py with ContentChunk Pydantic model matching data-model.md schema
- [X] T008 [P] Create backend/src/models/session.py with Session and ScopeContext Pydantic models matching data-model.md
- [X] T009 [P] Create backend/src/models/query.py with ChatRequest, ChatResponse, Citation, and Error Pydantic models matching openapi.yaml
- [X] T010 Implement Qdrant client initialization in backend/src/services/retrieval.py with connection validation
- [X] T011 [P] Implement Neon Postgres connection pool in backend/src/services/session.py with async support
- [X] T012 Create database migration script backend/src/scripts/migrate.py to create sessions and query_logs tables per data-model.md
- [X] T013 Implement rate limiting middleware in backend/src/api/middleware.py (60 req/min per session)
- [X] T014 [P] Implement query logging service in backend/src/services/logging.py for analytics (no PII)
- [X] T015 Create backend/src/scripts/embed_content.py for markdown parsing, chunking, embedding, and Qdrant upsert

**Checkpoint**: Foundation ready - Run `python -m src.scripts.migrate` and `python -m src.scripts.embed_content` successfully

---

## Phase 3: User Story 1 - Ask Question About Current Content (Priority: P1)

**Goal**: Learner asks a question and receives an accurate answer with citations from textbook content

**Independent Test**: Load any page, ask a question, verify response is accurate with correct source citation

### Implementation for User Story 1

- [X] T016 [US1] Implement EmbeddingService in backend/src/services/embedding.py with OpenAI text-embedding-3-small
- [X] T017 [US1] Implement RetrievalService.search() in backend/src/services/retrieval.py with vector search and page scope filter
- [X] T018 [US1] Implement ChatKitServer class in backend/src/services/chatkit_server.py extending ChatKitServer base
- [X] T019 [US1] Implement respond() method in chatkit_server.py: embed query → retrieve chunks → generate response with citations → stream
- [X] T020 [US1] Implement citation formatting in chatkit_server.py per Citation schema (chapter_title, section_title, paragraph_number, excerpt)
- [X] T021 [US1] Create POST /api/chat endpoint in backend/src/api/routes.py with SSE streaming response
- [X] T022 [US1] Add out-of-scope query detection in chatkit_server.py to return "I can only answer questions about the textbook content"
- [X] T023 [P] [US1] Create ChatbotToggle component in AIdd-book/src/components/ChatbotToggle/index.tsx with button and icon
- [X] T024 [P] [US1] Create ChatbotToggle styles in AIdd-book/src/components/ChatbotToggle/styles.module.css
- [X] T025 [US1] Create ChatbotPanel component in AIdd-book/src/components/ChatbotPanel/index.tsx with panel container
- [X] T026 [P] [US1] Create ChatMessage component in AIdd-book/src/components/ChatbotPanel/ChatMessage.tsx for message display with citations
- [X] T027 [P] [US1] Create ChatInput component in AIdd-book/src/components/ChatbotPanel/ChatInput.tsx with textarea and submit button
- [X] T028 [US1] Create chatApi service in AIdd-book/src/services/chatApi.ts with SSE handling for POST /api/chat
- [X] T029 [US1] Create useChatbot hook in AIdd-book/src/hooks/useChatbot.ts for state management (messages, loading, error)
- [X] T030 [US1] Swizzle DocItem Layout in AIdd-book/src/theme/DocItem/Layout/index.tsx to inject ChatbotToggle and ChatbotPanel
- [X] T031 [US1] Create ChatbotPanel styles in AIdd-book/src/components/ChatbotPanel/styles.module.css
- [X] T032 [US1] Add chatbot-icon.svg to AIdd-book/static/chatbot-icon.svg
- [X] T033 [US1] Add page scope detection in useChatbot.ts using useLocation() to extract current chapter_id

**Checkpoint**: US1 complete - Can ask questions on any page and receive cited answers

---

## Phase 4: User Story 2 - Scope Query to Selected Text (Priority: P2)

**Goal**: Learner highlights text and asks chatbot to explain it, response grounded in selection

**Independent Test**: Highlight text, ask question, verify response references only selected content

### Implementation for User Story 2

- [X] T034 [US2] Create useTextSelection hook in AIdd-book/src/hooks/useTextSelection.ts to capture highlighted text
- [X] T035 [US2] Add selection scope type to ScopeSelector in AIdd-book/src/components/ChatbotPanel/ScopeSelector.tsx
- [X] T036 [US2] Update chatApi.ts to include selected_text in scope when type="selection"
- [X] T037 [US2] Update RetrievalService.search() to handle selection scope: embed selected text → match against chunks
- [X] T038 [US2] Add visual indicator in ChatbotPanel when text is selected (show selection badge)
- [X] T039 [US2] Update useChatbot.ts to integrate useTextSelection and pass selection to scope

**Checkpoint**: US2 complete - Can highlight text and ask questions about it specifically

---

## Phase 5: User Story 3 - Multi-Turn Conversation with Context (Priority: P3)

**Goal**: Learner asks follow-up questions, chatbot maintains conversation context within session

**Independent Test**: Ask sequence of related questions, verify each response acknowledges prior context

### Implementation for User Story 3

- [X] T040 [US3] Implement SessionService.create() in backend/src/services/session.py to create new session row
- [X] T041 [US3] Implement SessionService.get() to retrieve session with conversation_history
- [X] T042 [US3] Implement SessionService.update() to append query/response to conversation_history (max 20)
- [X] T043 [US3] Implement SessionService.cleanup() to delete sessions inactive >24 hours
- [X] T044 [US3] Create POST /api/sessions endpoint in routes.py per openapi.yaml
- [X] T045 [US3] Create GET /api/sessions/{id} endpoint in routes.py
- [X] T046 [US3] Create DELETE /api/sessions/{id} endpoint in routes.py
- [X] T047 [US3] Update ChatKitServer.respond() to include conversation_history in context for multi-turn
- [X] T048 [US3] Update chatApi.ts to persist session_id in localStorage and send with requests
- [X] T049 [US3] Update useChatbot.ts to create session on first message if none exists
- [X] T050 [US3] Add session expiration handling in useChatbot.ts (detect 404, create new session)

**Checkpoint**: US3 complete - Can have multi-turn conversations, sessions expire after 24h

---

## Phase 6: User Story 4 - Scope Query to Module or Chapter (Priority: P4)

**Goal**: Learner selects module/chapter scope from dropdown, queries limited to that content

**Independent Test**: Select module scope, ask question, verify response references only that module

### Implementation for User Story 4

- [X] T051 [US4] Create ScopeSelector dropdown in AIdd-book/src/components/ChatbotPanel/ScopeSelector.tsx with options: Page, Selection, Module, All
- [X] T052 [US4] Fetch module list for ScopeSelector (static list from config or dynamic from API)
- [X] T053 [US4] Update useChatbot.ts to track selected scope and pass to chatApi
- [X] T054 [US4] Update RetrievalService.search() to filter by module_id when scope.type="module"
- [X] T055 [US4] Update scope on page navigation: auto-switch to "page" scope when navigating
- [X] T056 [US4] Add scope indicator to ChatbotPanel header showing current scope selection

**Checkpoint**: US4 complete - Can scope queries to any module/chapter from dropdown

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Quality gates, accessibility, performance, and documentation

### Tests

- [X] T057 [P] Create backend/tests/unit/test_retrieval.py with tests for vector search, filtering, and empty results
- [X] T058 [P] Create backend/tests/unit/test_session.py with tests for session lifecycle and expiration
- [X] T059 [P] Create backend/tests/unit/test_embedding.py with tests for chunking and metadata extraction
- [X] T060 [P] Create backend/tests/integration/test_chat_flow.py with end-to-end query processing test
- [X] T061 [P] Create backend/tests/contract/test_api.py with OpenAPI schema validation tests

### Accessibility

- [X] T062 Add ARIA labels to ChatbotPanel, ChatInput, ChatMessage components
- [X] T063 Implement keyboard navigation: Tab through elements, Enter to submit, Escape to close panel
- [X] T064 Add focus management: auto-focus input on panel open, trap focus in panel
- [X] T065 Add screen reader announcements for new messages (aria-live region)
- [X] T066 Run axe-core accessibility audit and fix any violations

### Performance & Quality

- [X] T067 Add loading indicator in ChatbotPanel during query processing
- [X] T068 Implement error handling UI with retry button for failed queries
- [X] T069 Validate page load impact <500ms by measuring with and without chatbot
- [X] T070 Run Docusaurus build validation: npm run build must complete without errors
- [X] T071 Create 100 sample test queries and evaluate 95% accuracy target
- [X] T072 Measure p95 latency and verify <3s target

### Documentation

- [X] T073 [P] Update AIdd-book/docusaurus.config.ts with chatbot environment configuration
- [X] T074 Verify quickstart.md instructions work for fresh setup

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - MVP
- **User Story 2 (Phase 4)**: Depends on US1 (reuses ChatbotPanel, chatApi)
- **User Story 3 (Phase 5)**: Depends on US1 (adds session layer)
- **User Story 4 (Phase 6)**: Depends on US1 (adds scope layer)
- **Polish (Phase 7)**: Depends on all user stories

### User Story Dependencies

```
        ┌─────────────────┐
        │  Foundational   │
        │    (Phase 2)    │
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │   User Story 1  │ ◄── MVP
        │     (Phase 3)   │
        └────────┬────────┘
                 │
       ┌─────────┼─────────┐
       ▼         ▼         ▼
┌──────────┐ ┌──────────┐ ┌──────────┐
│   US2    │ │   US3    │ │   US4    │
│ Selection│ │ Sessions │ │  Module  │
└──────────┘ └──────────┘ └──────────┘
       │         │         │
       └─────────┴─────────┘
                 │
                 ▼
        ┌─────────────────┐
        │     Polish      │
        │    (Phase 7)    │
        └─────────────────┘
```

### Within Each User Story

- Models before services (T007-T009 before T010-T015)
- Services before endpoints (T016-T022 before T023-T033)
- Backend before frontend (API first, then UI)

### Parallel Opportunities

**Phase 1 (Setup)**:
```bash
# Can run in parallel:
T002 [P] Create requirements.txt
T003 [P] Create .env.example
T004 [P] Create config.py
T006 [P] Create Dockerfile
```

**Phase 2 (Foundational)**:
```bash
# Can run in parallel:
T008 [P] Create session.py models
T009 [P] Create query.py models
T011 [P] Create Postgres connection
T014 [P] Create logging service
```

**Phase 3 (US1)**:
```bash
# Backend can run in parallel with frontend setup:
T023 [P] Create ChatbotToggle
T024 [P] Create ChatbotToggle styles
T026 [P] Create ChatMessage
T027 [P] Create ChatInput
```

**Phase 7 (Polish)**:
```bash
# All test files can run in parallel:
T057 [P] test_retrieval.py
T058 [P] test_session.py
T059 [P] test_embedding.py
T060 [P] test_chat_flow.py
T061 [P] test_api.py
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test end-to-end chat flow
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Complete Polish → Final release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (backend)
   - Developer B: User Story 1 (frontend)
3. After US1:
   - Developer A: User Story 3 (sessions)
   - Developer B: User Story 2 (selection) + User Story 4 (module scope)
4. All: Polish phase

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
