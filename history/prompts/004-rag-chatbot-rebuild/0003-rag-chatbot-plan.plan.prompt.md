---
id: "0003"
title: "RAG Chatbot Implementation Plan"
stage: plan
date: 2025-12-12
surface: agent
model: claude-opus-4-5-20251101
feature: 004-rag-chatbot-rebuild
branch: 004-rag-chatbot-rebuild
user: system
command: /sp.plan
labels: ["planning", "rag", "chatbot", "architecture", "openai-agents", "chatkit"]
links:
  spec: specs/004-rag-chatbot-rebuild/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot-rebuild/research.md
  - specs/004-rag-chatbot-rebuild/data-model.md
  - specs/004-rag-chatbot-rebuild/contracts/openapi.yaml
  - specs/004-rag-chatbot-rebuild/quickstart.md
  - specs/004-rag-chatbot-rebuild/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan - Execute the implementation planning workflow using the plan template to generate design artifacts.

Requirements:
- Complete rebuild of RAG chatbot backend
- Use OpenAI Agents SDK (NOT Assistant API) for agent logic
- Use OpenAI ChatKit for frontend/backend integration
- Use UV package manager for Python dependencies
- Use FastAPI as the web framework
- Use Qdrant Cloud for vector storage
- Use Neon Serverless PostgreSQL for relational data
- Re-embed all textbook markdown files
- MANDATORY: Review documentation via Context7 MCP server

## Response snapshot

Completed full planning workflow with Phase 0 (Research) and Phase 1 (Design):

**Phase 0 - Research (research.md)**:
- 8 technical decisions documented with code patterns
- OpenAI Agents SDK: `Runner.run_streamed()`, `@function_tool` decorator
- OpenAI ChatKit: `ChatKitServer`, `stream_agent_response()`
- Qdrant: `QdrantClient`, collection config, payload filtering
- Neon: SQLAlchemy ORM, Alembic migrations
- UV package manager setup
- Heading-based chunking strategy
- Circuit breaker error handling

**Phase 1 - Design**:
- data-model.md: PostgreSQL entities (Conversation, Message, QueryLog) + Qdrant TextbookChunk
- contracts/openapi.yaml: OpenAPI 3.1.0 spec with 7 endpoints
- quickstart.md: Developer setup guide with 8-step walkthrough

**plan.md**:
- Technical context filled with all specifications
- Constitution check passed
- Project structure defined (web application layout)
- 10 key technical decisions documented
- Implementation phases outlined
- Risks and mitigations identified

## Outcome

- Impact: Complete planning artifacts generated; ready for /sp.tasks to generate implementation breakdown
- Tests: N/A (planning phase)
- Files: 5 files created/modified (research.md, data-model.md, openapi.yaml, quickstart.md, plan.md)
- Next prompts: `/sp.tasks` to generate implementation task breakdown
- Reflection: Comprehensive planning with Context7 documentation review ensured accurate API patterns; all NEEDS CLARIFICATION items from spec resolved

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation - PASS (all artifacts generated, constitution check passed)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
