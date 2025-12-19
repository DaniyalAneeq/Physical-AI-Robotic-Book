---
id: 0003
title: Authentication Fix Implementation Planning
stage: plan
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-fix-auth
branch: 001-fix-auth
user: Gemini
command: /sp.plan
labels: ["planning", "architecture", "implementation", "authentication", "session-management"]
links:
  spec: specs/001-fix-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-fix-auth/plan.md (created, 1018 lines)
tests:
  - Constitution compliance validation (passed with grandfathering note)
  - Project structure validation (13 files identified)
  - Phase 0 research task completeness
  - Phase 1 artifact completeness (data model, API contracts, quickstart guide)
---

## Prompt

(No additional arguments provided)

User executed: `/sp.plan`

## Response snapshot

Successfully created comprehensive technical implementation plan with embedded Phase 0 and Phase 1 artifacts.

**Key deliverables:**
1. **Technical Context**: Documented technology stack (Python 3.11+, TypeScript 5.6.2, FastAPI, React, Neon Postgres) and environment configuration (dev vs prod)

2. **Constitution Check**: Evaluated Section IX "Authentication MUST be implemented using Better Auth" requirement:
   - Decision: Grandfather existing FastAPI auth system
   - Rationale: Project's auth predates constitution requirement, spec excludes Better Auth migration from scope
   - Future requirement: New auth features must use Better Auth per constitution

3. **Project Structure**: Identified 13 files requiring modification:
   - Backend (7 files): auth.py, oauth.py, session_service.py, user_service.py, config.py, main.py, tests/
   - Frontend (3 files): useAuth.tsx, authApi.ts, callback.tsx
   - E2E (3 files): auth.spec.ts, oauth.spec.ts, session.spec.ts

4. **Phase 0 Research** (embedded in plan.md):
   - Cross-origin cookie configuration (SameSite=Lax for dev, None+Secure for prod)
   - CORS configuration for credentials (explicit origins, no wildcard)
   - OAuth token exchange best practices (state verification, session before redirect)
   - FastAPI session management patterns (cookie-based, 30-day expiration)

5. **Phase 1 Data Model** (embedded in plan.md):
   - 4 entities documented: User, Session, OAuth Account, Onboarding Profile
   - Field specifications with types, constraints, relationships
   - No schema changes required (existing tables compatible)

6. **Phase 1 API Contracts** (embedded in plan.md):
   - OpenAPI 3.0 specification for 8 endpoints:
     - POST /auth/register (email/password signup)
     - POST /auth/login (email/password login)
     - POST /auth/logout (session invalidation)
     - GET /auth/session (session validation)
     - GET /auth/oauth/google (OAuth initiation)
     - GET /auth/oauth/callback (OAuth callback)
     - POST /auth/exchange-token (OAuth token exchange)
     - GET /auth/me (current user retrieval)
   - Cookie contract documentation (HttpOnly, SameSite, Secure, max-age)

7. **Phase 1 Quickstart Guide** (embedded in plan.md):
   - Developer setup checklist (environment variables, dependencies, database)
   - Verification checklist (cookie attributes, CORS headers, session persistence)
   - Debugging guide (browser DevTools, backend logs, common issues)

**Technical Decisions:**
- Cookie SameSite strategy: Environment-specific (Lax for HTTP dev, None for HTTPS prod)
- No database migrations required (existing schema sufficient)
- Minimal changes: Configuration fixes + OAuth verification step + onboarding trigger logic
- Test-first implementation: Red-Green-Refactor cycle for each task

**Plan Structure:**
- Embedded all Phase 0 and Phase 1 artifacts within plan.md (1018 lines)
- Phase 2 reserved for /sp.tasks command output (granular task list)
- All research findings and architectural decisions documented in single consolidated file

## Outcome

- ✅ Impact: Comprehensive implementation plan ready for task generation. All Phase 0 research and Phase 1 artifacts embedded in plan.md, covering 13 files across backend, frontend, and E2E tests.
- 🧪 Tests: Constitution compliance validated (grandfathered FastAPI auth). Project structure validated (7 backend, 3 frontend, 3 E2E files identified). Phase 0/1 completeness confirmed.
- 📁 Files: Created specs/001-fix-auth/plan.md (1018 lines) with embedded research, data model, API contracts, and quickstart guide.
- 🔁 Next prompts: `/sp.tasks` to generate granular task list (30-40 tasks) for Test-Driven Development implementation
- 🧠 Reflection: Embedding Phase 0 and Phase 1 content within plan.md (rather than separate files) creates a single source of truth for all planning artifacts. This approach simplifies navigation and ensures all architectural decisions are co-located with implementation guidance.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow executed successfully with all Phase 0 and Phase 1 artifacts embedded.
- Graders run and results (PASS/FAIL): Constitution compliance PASSED (with grandfathering justification), Project structure validation PASSED (13 files identified), Artifact completeness PASSED (research, data model, contracts, quickstart all embedded)
- Prompt variant (if applicable): Standard /sp.plan workflow with embedded artifact strategy
- Next experiment (smallest change to try): Consider whether embedded artifact approach (all in plan.md) vs separate files (research.md, contracts/, etc.) provides better developer experience for larger features with 50+ tasks
