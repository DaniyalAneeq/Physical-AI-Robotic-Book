# Tasks: RAG Chatbot Full Rebuild

**Input**: Design documents from `/specs/004-rag-chatbot-rebuild/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/openapi.yaml

**Tests**: Not explicitly requested in specification - tests are omitted per task generation rules.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `rag-chatbot/backend/` (Python/FastAPI)
- **Frontend**: `AIdd-book/src/components/` (Docusaurus/React)
- Scripts: `rag-chatbot/backend/scripts/`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create project structure and initialize dependencies

- [ ] T001 Create backend project structure with UV at rag-chatbot/backend/
- [ ] T002 Initialize pyproject.toml with FastAPI, openai-agents, chatkit-python, qdrant-client, sqlalchemy, alembic, psycopg dependencies in rag-chatbot/backend/pyproject.toml
- [ ] T003 [P] Create .env.example with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL placeholders in rag-chatbot/backend/.env.example
- [ ] T004 [P] Create app package structure with __init__.py files in rag-chatbot/backend/app/, app/api/, app/models/, app/services/, app/tools/
- [ ] T005 [P] Configure ruff linting and formatting in rag-chatbot/backend/pyproject.toml

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create config.py with environment variable loading and settings class in rag-chatbot/backend/app/config.py
- [ ] T007 Create database.py with SQLAlchemy engine, session factory, and Base in rag-chatbot/backend/app/models/database.py
- [ ] T008 [P] Create Conversation model per data-model.md in rag-chatbot/backend/app/models/conversation.py
- [ ] T009 [P] Create Message model with FK to Conversation per data-model.md in rag-chatbot/backend/app/models/message.py
- [ ] T010 [P] Create QueryLog model per data-model.md in rag-chatbot/backend/app/models/query_log.py
- [ ] T011 Initialize Alembic and create initial migration per data-model.md in rag-chatbot/backend/alembic/
- [ ] T012 Create main.py FastAPI app skeleton with CORS and router mounts in rag-chatbot/backend/app/main.py
- [ ] T013 [P] Create health check endpoint returning service status in rag-chatbot/backend/app/api/health.py
- [ ] T014 Create vector_store.py with Qdrant client setup and circuit breaker in rag-chatbot/backend/app/services/vector_store.py
- [ ] T015 [P] Create init_qdrant.py script to initialize collection with payload indexes in rag-chatbot/backend/scripts/init_qdrant.py
- [ ] T016 Create embeddings.py service with text-embedding-3-small integration in rag-chatbot/backend/app/services/embeddings.py
- [ ] T017 Create chunker.py with heading-based markdown chunking logic per research.md in rag-chatbot/backend/app/services/chunker.py
- [ ] T018 Create index_textbook.py script to process AIdd-book/docs/ and populate Qdrant in rag-chatbot/backend/scripts/index_textbook.py
- [ ] T019 [P] Create index management endpoints (status, rebuild) in rag-chatbot/backend/app/api/index.py

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask Questions About Textbook Content (Priority: P1)

**Goal**: Enable users to ask questions and receive accurate, grounded answers from textbook content with citations

**Independent Test**: Ask "What is URDF and how is it used in robotics?" and verify response cites relevant textbook sections and delivers accurate information within 3 seconds

### Implementation for User Story 1

- [ ] T020 [US1] Create retriever.py with @function_tool decorator for Qdrant RAG retrieval in rag-chatbot/backend/app/tools/retriever.py
- [ ] T021 [US1] Create agent.py with TextbookAssistant Agent configuration and system instructions in rag-chatbot/backend/app/services/agent.py
- [ ] T022 [US1] Implement ChatKitServer subclass with respond() method using Runner.run_streamed() in rag-chatbot/backend/app/api/chatkit.py
- [ ] T023 [US1] Wire /chatkit POST endpoint with SSE streaming response in rag-chatbot/backend/app/api/chatkit.py
- [ ] T024 [US1] Add citation extraction and formatting in agent response handling in rag-chatbot/backend/app/services/agent.py
- [ ] T025 [US1] Add out-of-scope query detection and polite redirection in agent instructions in rag-chatbot/backend/app/services/agent.py
- [ ] T026 [US1] Add query logging to QueryLog table on each request in rag-chatbot/backend/app/api/chatkit.py
- [ ] T027 [US1] Create TextbookChat React component with useChatKit hook in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T028 [US1] Create styles.module.css for TextbookChat component layout in AIdd-book/src/components/TextbookChat/styles.module.css
- [ ] T029 [US1] Add ChatKit npm dependency to AIdd-book/package.json

**Checkpoint**: User Story 1 complete - users can ask questions and receive grounded answers with citations

---

## Phase 4: User Story 2 - Multi-Turn Conversations (Priority: P2)

**Goal**: Enable follow-up conversations where the chatbot remembers previous questions

**Independent Test**: Ask "What is ROS 2?" followed by "How do I install it?" and verify the second response understands "it" refers to ROS 2

### Implementation for User Story 2

- [ ] T030 [US2] Implement conversation creation on first message in session in rag-chatbot/backend/app/api/chatkit.py
- [ ] T031 [US2] Add message persistence (user + assistant) to Message table in rag-chatbot/backend/app/api/chatkit.py
- [ ] T032 [US2] Implement conversation context loading for ChatKit thread in rag-chatbot/backend/app/api/chatkit.py
- [ ] T033 [US2] Add session_id extraction from ChatKit request in rag-chatbot/backend/app/api/chatkit.py
- [ ] T034 [US2] Configure Agent session management for multi-turn context in rag-chatbot/backend/app/services/agent.py
- [ ] T035 [US2] Add new conversation isolation (no context bleed) in rag-chatbot/backend/app/api/chatkit.py

**Checkpoint**: User Story 2 complete - multi-turn conversations with context preservation work independently

---

## Phase 5: User Story 3 - Real-Time Streaming Responses (Priority: P2)

**Goal**: Display chatbot responses word-by-word in real-time with <1 second first token

**Independent Test**: Time first-token appearance vs. complete response and verify tokens stream continuously without stuttering

### Implementation for User Story 3

- [ ] T036 [US3] Verify Runner.run_streamed() SSE event handling in ChatKit endpoint in rag-chatbot/backend/app/api/chatkit.py
- [ ] T037 [US3] Add streaming state indicators (typing, streaming) to ChatKit component in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T038 [US3] Implement graceful partial response preservation on network interruption in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T039 [US3] Add error message display for streaming failures in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T040 [US3] Optimize chunk size for smooth token flow in streaming in rag-chatbot/backend/app/api/chatkit.py

**Checkpoint**: User Story 3 complete - streaming responses with <1s first token latency work independently

---

## Phase 6: User Story 4 - Access Conversation History (Priority: P3)

**Goal**: Allow returning users to access and manage previous conversations

**Independent Test**: Have a conversation, refresh the page, and verify the previous conversation is accessible in the sidebar

### Implementation for User Story 4

- [ ] T041 [US4] Create conversations.py API router with CRUD operations in rag-chatbot/backend/app/api/conversations.py
- [ ] T042 [US4] Implement GET /api/conversations list endpoint with session_id filter in rag-chatbot/backend/app/api/conversations.py
- [ ] T043 [US4] Implement GET /api/conversations/{id} detail endpoint in rag-chatbot/backend/app/api/conversations.py
- [ ] T044 [US4] Implement PATCH /api/conversations/{id} for title updates in rag-chatbot/backend/app/api/conversations.py
- [ ] T045 [US4] Implement DELETE /api/conversations/{id} with cascade in rag-chatbot/backend/app/api/conversations.py
- [ ] T046 [US4] Add auto-title generation from first user message in rag-chatbot/backend/app/api/conversations.py
- [ ] T047 [US4] Enable ChatKit history sidebar with conversation list in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T048 [US4] Add conversation selection and loading in ChatKit component in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T049 [US4] Add rename and delete UI actions for conversations in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T050 [US4] Implement localStorage session_id persistence for anonymous users in AIdd-book/src/components/TextbookChat/index.tsx

**Checkpoint**: User Story 4 complete - conversation history with CRUD operations works independently

---

## Phase 7: User Story 5 - Module-Specific Queries (Priority: P3)

**Goal**: Allow users to filter queries to specific modules or chapters for focused answers

**Independent Test**: Ask about "sensor fusion" with Module 3 filter and verify responses come only from that module

### Implementation for User Story 5

- [ ] T051 [US5] Add module filter parameter to retriever tool in rag-chatbot/backend/app/tools/retriever.py
- [ ] T052 [US5] Implement Qdrant payload filtering by module in vector_store.py in rag-chatbot/backend/app/services/vector_store.py
- [ ] T053 [US5] Add module selection UI (dropdown/tabs) to ChatKit component in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T054 [US5] Pass module filter through ChatKit request to backend in AIdd-book/src/components/TextbookChat/index.tsx
- [ ] T055 [US5] Extract module filter from ChatKit context in chatkit.py in rag-chatbot/backend/app/api/chatkit.py
- [ ] T056 [US5] Add "All Modules" default option with no filter in AIdd-book/src/components/TextbookChat/index.tsx

**Checkpoint**: User Story 5 complete - module-specific filtering works independently

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T057 Add consistent error response format per openapi.yaml in rag-chatbot/backend/app/api/
- [ ] T058 [P] Implement 7-day conversation cleanup job in rag-chatbot/backend/scripts/cleanup.py
- [ ] T059 [P] Implement 30-day query log cleanup job in rag-chatbot/backend/scripts/cleanup.py
- [ ] T060 Add graceful shutdown handler for in-flight requests in rag-chatbot/backend/app/main.py
- [ ] T061 [P] Add responsive design styles for mobile viewports in AIdd-book/src/components/TextbookChat/styles.module.css
- [ ] T062 [P] Create .env.example for frontend with CHATKIT_API_URL in AIdd-book/.env.example
- [ ] T063 Run quickstart.md validation - verify all setup steps work end-to-end
- [ ] T064 Update AIdd-book Docusaurus config to include TextbookChat on appropriate pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 8)**: Depends on at least US1 being complete

### User Story Dependencies

| Story | Priority | Dependencies | Can Start After |
|-------|----------|--------------|-----------------|
| US1 - Ask Questions | P1 | Foundational | Phase 2 complete |
| US2 - Multi-Turn | P2 | Foundational, shares chatkit.py with US1 | Phase 2 complete |
| US3 - Streaming | P2 | Foundational, builds on US1 chatkit | Phase 2 complete |
| US4 - History | P3 | Foundational only | Phase 2 complete |
| US5 - Module Filter | P3 | Foundational, extends US1 retriever | Phase 2 complete |

**Note**: US2 and US3 share files with US1 but can be developed as incremental enhancements. US4 and US5 are largely independent.

### Within Each User Story

- Models before services (already in Foundational)
- Services before endpoints
- Backend before frontend integration
- Core implementation before polish

---

## Parallel Opportunities

### Phase 1 (Setup)
```bash
# All can run in parallel after T001, T002:
T003: Create .env.example
T004: Create package structure
T005: Configure linting
```

### Phase 2 (Foundational)
```bash
# After T006, T007, these can run in parallel:
T008: Conversation model
T009: Message model
T010: QueryLog model
T013: Health endpoint
T015: init_qdrant.py script
T019: Index endpoints
```

### User Stories (Phase 3+)
```bash
# With adequate staffing, after Phase 2:
Developer A: US1 (P1) - Core Q&A
Developer B: US4 (P3) - History (independent backend)
Developer C: US5 (P3) - Module filter (independent extension)

# Then US2/US3 build on US1
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T019)
3. Complete Phase 3: User Story 1 (T020-T029)
4. **STOP and VALIDATE**: Test core Q&A independently
5. Deploy/demo if ready - basic chatbot works!

### Incremental Delivery

| Increment | Stories | Cumulative Value |
|-----------|---------|------------------|
| MVP | US1 | Basic Q&A with citations |
| v1.1 | US1 + US2 | Multi-turn conversations |
| v1.2 | US1 + US2 + US3 | Smooth streaming UX |
| v1.3 | + US4 | Persistent history |
| v1.4 | + US5 | Module filtering |

### Parallel Team Strategy

With 3 developers:
1. **All**: Complete Setup + Foundational together
2. **Dev A**: US1 (core) → US2 (context) → US3 (streaming)
3. **Dev B**: US4 (history - backend heavy)
4. **Dev C**: US5 (filtering) → Polish tasks

---

## Task Summary

| Phase | Tasks | Parallel Opportunities |
|-------|-------|----------------------|
| Phase 1: Setup | 5 | 3 |
| Phase 2: Foundational | 14 | 7 |
| Phase 3: US1 - Ask Questions | 10 | 0 (sequential) |
| Phase 4: US2 - Multi-Turn | 6 | 0 (sequential) |
| Phase 5: US3 - Streaming | 5 | 0 (sequential) |
| Phase 6: US4 - History | 10 | 0 (sequential) |
| Phase 7: US5 - Module Filter | 6 | 0 (sequential) |
| Phase 8: Polish | 8 | 4 |
| **Total** | **64** | **14** |

### Per-Story Task Counts

| User Story | Tasks | Independent Test |
|------------|-------|------------------|
| US1 | 10 | Ask URDF question, verify citations |
| US2 | 6 | Follow-up "it" reference test |
| US3 | 5 | First-token timing test |
| US4 | 10 | Refresh and access history test |
| US5 | 6 | Module filter verification |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests not included as not explicitly requested in spec
