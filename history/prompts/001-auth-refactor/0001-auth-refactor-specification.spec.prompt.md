---
id: 0001
title: Auth Refactor Specification
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-auth-refactor
branch: 001-auth-refactor
user: developer
command: /sp.specify
labels: ["authentication", "backend-refactor", "onboarding", "oauth", "unified-server"]
links:
  spec: specs/001-auth-refactor/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-auth-refactor/spec.md
 - specs/001-auth-refactor/checklists/requirements.md
tests:
 - Specification validation (all checklist items passed)
---

## Prompt

Create a detailed specification to refactor and fix authentication integration in my existing project.

Project context:
- Monorepo structure: rag-chatbot/
- Backend: Python + FastAPI
- Frontend: Docusaurus (existing, decoupled, must not be restructured)
- Existing RAG chatbot backend already implemented
- Authentication implemented using Better Auth (via Claude Code) but currently mis-structured
- Current issue: Auth backend lives inside rag-chatbot/backend/auth_backend, causing redirection issues and requiring multiple servers

Core requirements:
1. Use a SINGLE FastAPI server (single port, e.g. :8000).
2. Auth and RAG must run on the SAME FastAPI app instance using routers (no separate servers).
3. Separate logical modules, NOT separate servers:
   - /api/* → rag-chatbot/backend (RAG chatbot backend)
   - /auth/* → Authentication (Better Auth)
4. Refactor directory structure so auth backend is a top-level backend module, not nested incorrectly.
5. No separate server processes or ports for auth and chatbot.
6. Fix all authentication issues:
   - OAuth redirects
   - login/register flow
   - token/session handling
7. Enforce authentication:
   - Unauthenticated users cannot access rag-chatbot (chat requires login).
   - Redirect unauthenticated users to login.
8. Registration + onboarding flow:
   - New user registers → automatically logged in
   - AFTER login, user must complete onboarding BEFORE accessing rag-chatbot
   - Onboarding form fields: user type, area of interest, experience level, topics of interest
   - Save onboarding data in database (Neon/Postgres) against the authenticated user
   - Only after onboarding completion, redirect user to base application URL
9. Existing RAG chatbot functionality must remain unchanged.
10. API-first architecture; frontend remains fully decoupled.

Output expectations:
- Clear directory structure (before vs after)
- Defined API routes and prefixes
- Auth flow diagrams (login → register → onboarding → app)
- Database changes (tables/fields)
- Assumptions and constraints
- Explicit non-goals

## Response snapshot

Generated comprehensive feature specification for authentication refactor with:

**User Stories (5 prioritized)**:
- P1: Unified Backend Architecture (single server on port 8000)
- P2: User Registration and Login (email/password + OAuth)
- P2: Mandatory Onboarding Flow (collect user profile data)
- P3: Protected RAG Chatbot Access (auth enforcement)
- P4: Session Management and Logout

**Functional Requirements**: 24 testable requirements covering:
- Unified backend architecture (single FastAPI app, route prefixes)
- Authentication flows (registration, login, OAuth, session management)
- Onboarding requirements (form fields, database storage, flow enforcement)
- Security enforcement (protected endpoints, redirects)
- Backwards compatibility (preserve RAG functionality)

**Success Criteria**: 12 measurable outcomes including:
- Single-command backend startup
- Registration in under 2 minutes
- Onboarding in under 3 minutes
- 100% redirect enforcement
- OAuth flow success
- No regression in RAG functionality

**Architecture Details**:
- Directory structure refactor (before/after diagrams)
- API route definitions (all routes on localhost:8000)
- Authentication flow diagrams (registration, login, protected access)
- Database schema changes (onboarding_profiles table, users table modification)

**Scope Management**:
- 10 assumptions (Better Auth compatibility, database sharing, etc.)
- 10 constraints (no frontend restructuring, single port, etc.)
- 12 non-goals (email verification, password reset, MFA, etc.)
- Dependencies (external, internal, feature)

## Outcome

- ✅ Impact: Created production-ready specification for authentication refactor. Spec clearly separates business requirements from implementation, defines measurable success criteria, and provides architectural clarity for planning phase.
- 🧪 Tests: Specification validation checklist created and all items passed (requirements are testable, success criteria are measurable, no implementation details leaked)
- 📁 Files: specs/001-auth-refactor/spec.md (546 lines), specs/001-auth-refactor/checklists/requirements.md
- 🔁 Next prompts: Ready for `/sp.plan` to generate architectural plan, then `/sp.tasks` to break down into implementation tasks
- 🧠 Reflection: Spec successfully captures complex authentication refactor with multiple interacting concerns (unified backend, OAuth, onboarding, security). Clear prioritization enables incremental implementation. No clarifications needed - all requirements well-defined.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification is complete and unambiguous
- Graders run and results (PASS/FAIL): Specification quality checklist PASSED (all items validated)
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Proceed to planning phase to validate that spec provides sufficient detail for architectural decisions
