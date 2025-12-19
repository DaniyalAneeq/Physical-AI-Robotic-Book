---
id: 0002
title: i18n-implementation-planning
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: 002-i18n
branch: 002-i18n
user: agent
command: /sp.plan
labels: ["i18n", "localization", "implementation-plan", "architecture", "rag", "docusaurus"]
links:
  spec: /mnt/d/AIDD-hackathon-book/AI-robotics/specs/002-i18n/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-i18n/plan.md
 - specs/002-i18n/research.md
 - specs/002-i18n/data-model.md
 - specs/002-i18n/quickstart.md
 - specs/002-i18n/contracts/api-spec.yaml
 - CLAUDE.md (agent context update)
tests:
 - Constitution compliance check (all gates PASS)
 - Technology research validation (10/10 decisions made)
---

## Prompt

Create an implementation plan for adding English ↔ Urdu i18n support to an authenticated AI-powered book platform.

Input:
- Approved specification for i18n + localization (Docusaurus + FastAPI + Qdrant + RAG)
- Translation strategy: pre-translated book content + on-demand fallback
- Locale handling via Accept-Language header (en-US, ur-PK)

Plan Requirements:
- Break work into clear phases (Frontend, Backend, Data, RAG, Infra)
- Define execution order and dependencies
- Identify migration steps for existing English-only content
- Include caching and performance considerations
- Include auth-gated Urdu toggle rollout
- Highlight risks and mitigations (latency, terminology drift, RTL issues)

Output:
- Step-by-step implementation plan
- Clear milestones with validation checkpoints
- Ready to be decomposed into tasks for /sp.tasks

## Response snapshot

Created comprehensive 6-phase implementation plan for bilingual platform support:

**Phase 0: Research & Technology Validation** ✅
- Resolved 10 critical technology decisions
- Documented in research.md with alternatives and rationale
- Key decisions: Docusaurus i18n, langdetect library, OpenAI GPT-4 translation, PostgreSQL cache

**Phase 1: Data Model & API Contracts** ✅
- Designed PostgreSQL schema extensions (users.preferred_locale, translation_cache table)
- Designed Qdrant dual-language payload structure (content_en, content_ur, translation_status)
- Created OpenAPI 3.0 specification (4 endpoints: preferences, content, chatbot)
- Non-destructive migration strategy for existing English content
- Documented in data-model.md, contracts/api-spec.yaml, quickstart.md

**Phase 2: Implementation Planning** (This Document) ✅
- 6-phase execution timeline (Weeks 1-4)
- Dependency mapping: Database → Frontend/Backend → RAG → Content → Testing
- Performance optimization: Translation cache (80% hit rate target), RTL CSS strategy
- Auth-gated rollout: Better Auth integration, staged beta (5% → 100%)
- Risk mitigation: 5 major risks with contingency plans
- 6 validation checkpoints with measurable success criteria

**Technical Architecture**:
- **Frontend**: Docusaurus native i18n, RTL layout, Language Toggle component (auth-gated)
- **Backend**: FastAPI locale-aware endpoints, Accept-Language header parsing, translation cache service
- **Data**: PostgreSQL extensions, Qdrant dual-language payloads, SHA-256 cache keys
- **RAG**: langdetect for query language detection, OpenAI GPT-4 for translation, technical term preservation

**Constitution Compliance**: ✅ PASS
- Authentication gates satisfied (Better Auth integration, no custom auth logic)
- RAG quality gates met (90%+ accuracy target, < 500ms latency, caching strategy)
- Technology currency maintained (no unauthorized dependencies, langdetect only addition)

**Execution Order**:
```
Week 1: Database Schema Updates → Verification
Week 2: Frontend UI (parallel) + Backend API → Integration
Week 3: RAG Chatbot Translation → Cache validation
Week 4: Content Ingestion → Testing → Beta Launch (5%) → Full Rollout (100%)
```

**Key Performance Targets** (from spec):
- Language toggle: < 2s
- Content retrieval: < 200ms
- Chatbot translation overhead: < 500ms
- Cache hit rate: > 80%

## Outcome

- ✅ Impact: Complete implementation plan with 4 planning artifacts (plan.md, research.md, data-model.md, quickstart.md) + API contracts. Zero unknowns remaining - all technology decisions validated. Ready for task decomposition.
- 🧪 Tests: Constitution compliance validated (all 14 quality gates PASS). Technology research completed (10/10 decisions documented). No validation tests run yet (implementation phase).
- 📁 Files: 5 files created/updated (plan.md, research.md, data-model.md, quickstart.md, contracts/api-spec.yaml, CLAUDE.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable task list from this plan. Tasks will decompose 6 phases into specific implementation steps with acceptance criteria.
- 🧠 Reflection: Successfully balanced technical depth (database schemas, API contracts, caching strategies) with practical implementation guidance (quickstart with week-by-week timeline, rollback procedures, troubleshooting). Risk mitigation comprehensive - identified 5 major risks with contingency plans. Constitution compliance proactively validated - no blockers for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning phases completed successfully
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all quality gates satisfied)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (Research) + Phase 1 (Data Model & Contracts) completion
- Next experiment (smallest change to try): Proceed to task generation with `/sp.tasks` to validate plan decomposability and identify any missing implementation details
