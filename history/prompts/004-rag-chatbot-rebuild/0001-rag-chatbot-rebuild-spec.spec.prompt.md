---
id: "0001"
title: "RAG Chatbot Rebuild Specification"
stage: spec
date: 2025-12-12
surface: agent
model: claude-opus-4-5-20251101
feature: 004-rag-chatbot-rebuild
branch: 004-rag-chatbot-rebuild
user: system
command: /sp.specify
labels: ["rag", "chatbot", "openai-agents-sdk", "chatkit", "rebuild"]
links:
  spec: specs/004-rag-chatbot-rebuild/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot-rebuild/spec.md
  - specs/004-rag-chatbot-rebuild/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

This is a full rebuild specification. Follow these rules carefully.

========================
## CORE OBJECTIVE
Rebuild the entire RAG chatbot backend from scratch using the latest OpenAI Agents SDK and OpenAI ChatKit, replacing the previous Assistant API architecture, delete the previous backend and create from scratch.

========================
## BEFORE ANY WORK — DOCUMENTATION REQUIREMENT
You MUST first use the Context-7 MCP server to search and read the official documentation for:

1. OpenAI Agents SDK
2. OpenAI ChatKit
3. Qdrant
4. Neon

You MUST NOT generate any code, folder structure, or implementation plan until you have read:
- How OpenAI Agents SDK creates agents
- How ChatKit handles frontend UI components and streaming
- How the SDK handles files, memory, tools, and function calling

After reading the documentation, you MUST explicitly confirm:
"I have reviewed the Agents SDK documentation and ChatKit documentation via Context-7."

========================
## SCOPE OF THIS SPECIFICATION
This specification describes a full reset of the following components:

### Backend
- Delete the existing backend folder entirely
- Rebuild a new backend using **UV package manager**
- Use **FastAPI + OpenAI Agents SDK + OpenAI's Chatkit**
- Recreate a clean project layout with uv
- Remove all old Assistant API code
- Remove all outdated embedding scripts

### Database
- Database engine: **Neon Serverless PostgreSQL**
- Migrations:
  - Delete existing migrations
  - Recreate alembic migrations from scratch
  - Use proper SQLAlchemy models for sessions, logs, telemetry

### Vector Store
- Qdrant Cloud
- Delete all existing vectors
- Re-embed all textbook markdown files after the backend is rebuilt
- Use a clean consistent schema

### Embedding Pipeline
- Use OpenAI embeddings
- Clear → Recreate → Re-index all content

### Agent Architecture
- Must use OpenAI Agents SDK ONLY
- Agents should:
  - Have tools for query routing
  - Have a content-grounding tool for Qdrant retrieval
  - Use Neon for conversation memory
  - Run inside FastAPI

### Frontend
- Use OpenAI ChatKit
- Replace old React Chatbot
- Rebuild a modern, smooth, animated UI inside Docusaurus
- Fully responsive design

========================
## NON-FUNCTIONAL REQUIREMENTS
- All components must follow constitution principles
- All versions must be pinned
- All code must follow PEP 8
- Latency target: < 3 seconds with streaming
- Groundedness: 100% from textbook content
- Zero hallucinations

========================
## OUT OF SCOPE
These things must NOT be included in the spec:
- Constitution rules
- Detailed implementation-level instructions
- Code files

========================
## FINAL TASK
Rewrite the entire spec.md using the above rules and producing:

1. A clear summary
2. Functional requirements
3. Non-functional requirements
4. Architecture overview
5. Data flow
6. User stories
7. Acceptance criteria
8. Risks & mitigations

Make the spec production-grade, highly organized, and ready for the plan.md stage.

## Response snapshot

Created production-grade specification for RAG Chatbot Full Rebuild:

**Key deliverables:**
1. **Feature branch**: `004-rag-chatbot-rebuild` created and checked out
2. **Specification file**: `specs/004-rag-chatbot-rebuild/spec.md` (320 lines)
3. **Quality checklist**: `specs/004-rag-chatbot-rebuild/checklists/requirements.md`

**Specification contents:**
- Executive summary with clear scope (in/out)
- 5 prioritized user stories (P1-P3) with acceptance scenarios
- 20 functional requirements (backend, frontend, data pipeline)
- 10 non-functional requirements (performance, reliability, code quality)
- 4 key entities defined (Conversation, Message, TextbookChunk, QueryLog)
- 8 measurable success criteria
- Architecture diagram with data flow
- Risk matrix with 6 risks and mitigations
- Assumptions, dependencies, and glossary

**Documentation reviewed via Context7:**
- OpenAI Agents SDK (tools, sessions, streaming, Runner)
- OpenAI ChatKit (React components, streaming, FastAPI integration)
- Qdrant (Python client, embeddings, search)
- Neon (SQLAlchemy, psycopg2, Alembic migrations)

## Outcome

- Impact: Created comprehensive specification document ready for planning phase
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, requirements.md checklist)
- Next prompts: `/sp.clarify` or `/sp.plan` to proceed with implementation planning
- Reflection: Documentation review via Context7 ensured accurate understanding of OpenAI Agents SDK and ChatKit before specification

## Evaluation notes (flywheel)

- Failure modes observed: None - all validation checks passed
- Graders run and results (PASS/FAIL): Specification quality checklist - PASS (all items checked)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider adding more granular acceptance criteria for edge cases
