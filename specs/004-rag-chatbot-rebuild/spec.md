# Feature Specification: RAG Chatbot Full Rebuild

**Feature Branch**: `004-rag-chatbot-rebuild`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "Rebuild the entire RAG chatbot backend from scratch using the latest OpenAI Agents SDK and OpenAI ChatKit, replacing the previous Assistant API architecture."

---

## Clarifications

### Session 2025-12-12

- Q: How should users be identified without authentication? → A: No authentication for now; defer to later phase. Use anonymous browser-based sessions (localStorage UUID).
- Q: Which embedding model to use? → A: text-embedding-3-small (1536 dimensions, lower cost, faster).
- Q: How long should conversation history be retained? → A: 7 days (minimal storage, short-term reference).

---

## Executive Summary

This specification defines a complete rebuild of the RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI & Humanoid Robotics textbook. The rebuild replaces the existing OpenAI Assistant API architecture with the OpenAI Agents SDK and ChatKit framework, providing improved streaming, tool integration, and UI capabilities.

### Scope

**In Scope:**
- Complete backend rebuild using OpenAI Agents SDK
- New frontend chat interface using OpenAI ChatKit
- Fresh vector store setup with Qdrant Cloud
- New database schema with Neon Serverless PostgreSQL
- Re-embedding of all textbook content
- Session management and conversation history

**Out of Scope:**
- Multi-tenant architecture
- Analytics dashboard
- Admin interface
- Mobile native applications

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Textbook Content (Priority: P1)

As a learner studying the Physical AI & Humanoid Robotics textbook, I want to ask questions about the textbook content and receive accurate, grounded answers so that I can better understand complex topics.

**Why this priority**: This is the core value proposition of the chatbot - providing accurate, contextual answers from textbook content. Without this, the chatbot serves no purpose.

**Independent Test**: Can be fully tested by asking a question about ROS 2 navigation and verifying the response cites relevant textbook sections and delivers accurate information.

**Acceptance Scenarios**:

1. **Given** the user is on the textbook website, **When** they type "What is URDF and how is it used in robotics?", **Then** the chatbot returns a response grounded in the textbook's URDF chapter content within 3 seconds.

2. **Given** the user asks a question covered in multiple chapters, **When** the query is "How do I create a robot simulation?", **Then** the response synthesizes information from relevant sections (Gazebo, URDF, Isaac Sim) with proper citations.

3. **Given** the user asks a question not covered in the textbook, **When** they ask "What is the best pizza in New York?", **Then** the chatbot politely indicates this is outside its knowledge scope and redirects to textbook topics.

---

### User Story 2 - Multi-Turn Conversations (Priority: P2)

As a learner, I want to have follow-up conversations where the chatbot remembers my previous questions so that I can dive deeper into topics without repeating context.

**Why this priority**: Contextual conversations significantly improve learning experience but require the core Q&A functionality to be working first.

**Independent Test**: Can be tested by asking "What is ROS 2?" followed by "How do I install it?" and verifying the second response understands "it" refers to ROS 2.

**Acceptance Scenarios**:

1. **Given** a user has asked "What is a URDF file?", **When** they follow up with "Can you show me an example?", **Then** the chatbot provides a URDF example relevant to the previous question without the user re-specifying the topic.

2. **Given** a conversation has 5+ exchanges, **When** the user references something from the first message, **Then** the chatbot correctly retrieves and uses that context.

3. **Given** a user starts a new conversation, **When** they ask their first question, **Then** no previous conversation context interferes with the response.

---

### User Story 3 - Real-Time Streaming Responses (Priority: P2)

As a learner, I want to see the chatbot's response appear word-by-word in real-time so that I get immediate feedback and don't wait for complete responses.

**Why this priority**: Streaming dramatically improves perceived performance and user engagement, especially for longer responses.

**Independent Test**: Can be tested by timing first-token appearance vs. complete response and verifying tokens stream continuously.

**Acceptance Scenarios**:

1. **Given** a user submits a question, **When** processing begins, **Then** the first response token appears within 1 second.

2. **Given** a response is streaming, **When** tokens are received, **Then** they appear smoothly without visible chunking or delays between tokens.

3. **Given** a streaming response is in progress, **When** a network interruption occurs, **Then** the partial response remains visible and an error message appears.

---

### User Story 4 - Access Conversation History (Priority: P3)

As a returning learner, I want to access my previous conversations so that I can review past answers and continue from where I left off.

**Why this priority**: History provides continuity for learning but is not essential for initial chatbot functionality.

**Independent Test**: Can be tested by having a conversation, refreshing the page, and verifying the previous conversation is accessible.

**Acceptance Scenarios**:

1. **Given** a user has previous conversations, **When** they open the chatbot, **Then** they can see a list of their conversation history.

2. **Given** a user selects a previous conversation, **When** they click on it, **Then** the full conversation loads with all messages intact.

3. **Given** a user wants to organize conversations, **When** they rename or delete a conversation, **Then** the change persists across sessions.

---

### User Story 5 - Module-Specific Queries (Priority: P3)

As a learner studying a specific module, I want to filter my questions to a particular module or chapter so that I get more focused and relevant answers.

**Why this priority**: Module filtering enhances precision but requires the base RAG system to be fully functional first.

**Independent Test**: Can be tested by asking about "sensor fusion" with Module 3 filter and verifying responses come only from that module.

**Acceptance Scenarios**:

1. **Given** a user selects "Module 2: Digital Twin" filter, **When** they ask "How do I set up a simulation?", **Then** responses prioritize content from Module 2.

2. **Given** no module filter is selected, **When** a user asks a question, **Then** all modules are searched and the most relevant content is returned regardless of module.

---

### Edge Cases

- What happens when the vector store is temporarily unavailable?
  - System returns a graceful error message and suggests retrying
- How does the system handle extremely long questions (>2000 characters)?
  - Input is truncated with user notification; summarization is applied
- What happens when multiple users ask the same question simultaneously?
  - Responses are generated independently; no caching interference
- How does the system handle questions in non-English languages?
  - System responds in English, noting it's optimized for English content
- What happens if embedding generation fails during content indexing?
  - Failed chunks are logged; partial index is available; retry mechanism exists

---

## Requirements *(mandatory)*

### Functional Requirements

#### Backend Requirements

- **FR-001**: System MUST use OpenAI Agents SDK for all agent-based interactions
- **FR-002**: System MUST expose a FastAPI endpoint at `/chatkit` for ChatKit protocol communication
- **FR-003**: System MUST implement streaming responses using Server-Sent Events (SSE)
- **FR-004**: System MUST connect to Qdrant Cloud for vector similarity search
- **FR-005**: System MUST use OpenAI `text-embedding-3-small` (1536 dimensions) for content vectorization
- **FR-006**: System MUST store conversation sessions in Neon Serverless PostgreSQL with 7-day retention (automatic cleanup of older conversations)
- **FR-007**: System MUST implement a retrieval tool that queries Qdrant for relevant textbook chunks
- **FR-008**: System MUST include source citations (chapter, section) in responses
- **FR-009**: System MUST log all queries and responses for debugging and improvement
- **FR-010**: System MUST handle graceful degradation when external services are unavailable

#### Frontend Requirements

- **FR-011**: System MUST use OpenAI ChatKit React components for the chat interface
- **FR-012**: System MUST display streaming responses in real-time
- **FR-013**: System MUST integrate seamlessly within the Docusaurus textbook site
- **FR-014**: System MUST provide conversation history sidebar
- **FR-015**: System MUST support responsive design for desktop and mobile viewports
- **FR-016**: System MUST provide visual feedback during loading and streaming states

#### Data Pipeline Requirements

- **FR-017**: System MUST process all textbook markdown files from `AIdd-book/docs/`
- **FR-018**: System MUST chunk documents intelligently (by section/heading)
- **FR-019**: System MUST store metadata (module, chapter, section, file path) with each vector
- **FR-020**: System MUST support re-indexing without data loss during the process

### Non-Functional Requirements

- **NFR-001**: First response token MUST appear within 1 second of query submission
- **NFR-002**: Complete responses MUST be delivered within 3 seconds for typical queries
- **NFR-003**: System MUST handle 100 concurrent users without degradation
- **NFR-004**: All responses MUST be 100% grounded in textbook content (no hallucinations)
- **NFR-005**: Vector search latency MUST be under 200ms (p95)
- **NFR-006**: System MUST maintain 99.5% uptime
- **NFR-007**: All dependencies MUST have pinned versions
- **NFR-008**: All Python code MUST follow PEP 8 style guidelines
- **NFR-009**: All API responses MUST follow consistent error format
- **NFR-010**: System MUST support graceful shutdown with in-flight request completion

### Key Entities

- **Conversation**: Represents a chat session; contains session ID (browser-generated UUID), creation timestamp, and title. No user authentication required; sessions are anonymous and browser-local.
- **Message**: Individual exchange within a conversation; contains role (user/assistant), content, timestamp, and optional citations
- **TextbookChunk**: Indexed content unit; contains text, embedding vector, and metadata (module, chapter, section, file path)
- **QueryLog**: Audit record; contains query text, response summary, latency metrics, and timestamp

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive first response token within 1 second of submitting a question (measured via client-side timing)
- **SC-002**: 95% of user questions receive responses grounded in textbook content (measured via manual review of sample queries)
- **SC-003**: System supports 100 concurrent chat sessions without response degradation (measured via load testing)
- **SC-004**: Streaming responses maintain smooth token flow with no visible stuttering (measured via user testing)
- **SC-005**: Conversation history persists correctly across browser sessions (measured via functional testing)
- **SC-006**: All textbook content (100+ markdown files) is successfully indexed and searchable (measured via index completeness check)
- **SC-007**: Response relevance score averages 4+ out of 5 in user feedback (measured via optional feedback mechanism)
- **SC-008**: Zero responses contain information not present in the textbook (measured via content audit)

---

## Architecture Overview

### System Components

```
+------------------------------------------------------------------+
|                         Frontend                                  |
|  +------------------------------------------------------------+  |
|  |              Docusaurus Site (AIdd-book)                   |  |
|  |  +------------------------------------------------------+  |  |
|  |  |           OpenAI ChatKit React Component             |  |  |
|  |  |  - Streaming display                                 |  |  |
|  |  |  - Conversation history                              |  |  |
|  |  |  - Responsive UI                                     |  |  |
|  |  +------------------------------------------------------+  |  |
|  +------------------------------------------------------------+  |
+------------------------------------------------------------------+
                              |
                              | HTTPS (ChatKit Protocol)
                              v
+------------------------------------------------------------------+
|                         Backend                                   |
|  +------------------------------------------------------------+  |
|  |                   FastAPI Application                       |  |
|  |  +-----------------+  +-------------------------------+    |  |
|  |  |  /chatkit       |  |    OpenAI Agents SDK          |    |  |
|  |  |  Endpoint       |--|  - Textbook Agent             |    |  |
|  |  |  (SSE Stream)   |  |  - Retrieval Tool             |    |  |
|  |  +-----------------+  |  - Session Management         |    |  |
|  |                       +-------------------------------+    |  |
|  +------------------------------------------------------------+  |
+------------------------------------------------------------------+
           |                              |
           |                              |
           v                              v
+---------------------+      +---------------------------------+
|   Qdrant Cloud      |      |   Neon Serverless PostgreSQL    |
|  +---------------+  |      |  +-------------------------+    |
|  | Textbook      |  |      |  | conversations           |    |
|  | Vectors       |  |      |  | messages                |    |
|  | (embeddings)  |  |      |  | query_logs              |    |
|  +---------------+  |      |  +-------------------------+    |
+---------------------+      +---------------------------------+
```

### Data Flow

1. **Query Flow**:
   - User enters question in ChatKit interface
   - Request sent to FastAPI `/chatkit` endpoint
   - Agent SDK processes request, invokes retrieval tool
   - Retrieval tool queries Qdrant for relevant chunks
   - Agent generates response using retrieved context
   - Response streams back via SSE to ChatKit
   - Conversation saved to Neon PostgreSQL

2. **Indexing Flow**:
   - Markdown files read from `AIdd-book/docs/`
   - Content chunked by section/heading
   - Metadata extracted (module, chapter, section)
   - OpenAI embeddings generated for each chunk
   - Vectors upserted to Qdrant collection
   - Index completion logged

---

## Risks & Mitigations

| Risk                                         | Impact | Likelihood | Mitigation                                                     |
|----------------------------------------------|--------|------------|----------------------------------------------------------------|
| OpenAI API rate limits during indexing       | Medium | Medium     | Implement exponential backoff and batch processing             |
| Qdrant Cloud service outage                  | High   | Low        | Implement circuit breaker pattern; cache recent queries        |
| Response hallucination despite RAG           | High   | Low        | Strict system prompts; citation requirements; temperature=0    |
| Large markdown files cause chunking issues   | Medium | Medium     | Implement smart chunking with overlap; max chunk size limits   |
| Frontend-backend protocol mismatch           | Medium | Low        | Use official ChatKit libraries; comprehensive integration tests|
| Database migration failures                  | Medium | Low        | Use Alembic migrations; test on staging first; rollback procedures |

---

## Assumptions

- OpenAI API key with sufficient quota is available
- Qdrant Cloud account and cluster are provisioned
- Neon PostgreSQL database is provisioned
- Textbook content is complete and in markdown format
- UV package manager is available in the development environment
- Node.js 18+ is available for frontend build

---

## Dependencies

### External Services
- OpenAI API (GPT-4o, text-embedding-3-small/large)
- Qdrant Cloud (vector database)
- Neon Serverless PostgreSQL (relational database)

### Internal Dependencies
- Textbook content in `AIdd-book/docs/`
- Existing Docusaurus site structure

---

## Glossary

- **RAG**: Retrieval-Augmented Generation - technique that enhances LLM responses with retrieved relevant content
- **Agents SDK**: OpenAI's framework for building AI agents with tools, handoffs, and sessions
- **ChatKit**: OpenAI's UI framework for building chat interfaces with streaming support
- **SSE**: Server-Sent Events - protocol for streaming data from server to client
- **Embedding**: Dense vector representation of text for semantic similarity search
- **Qdrant**: Vector database optimized for similarity search
- **Neon**: Serverless PostgreSQL platform with autoscaling
