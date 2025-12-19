---
id: 0001
title: Authentication Session Persistence Fix Specification
stage: spec
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-fix-auth
branch: 001-fix-auth
user: Claude Code
command: /sp.specify
labels: ["authentication", "session-management", "cookies", "cors", "oauth", "bug-fix", "specification"]
links:
  spec: specs/001-fix-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-fix-auth/spec.md
  - specs/001-fix-auth/checklists/requirements.md
tests:
  - Specification quality validation (all checks passed)
---

## Prompt

Create a detailed specification to fix all authentication issues in the Physical-AI-Robotic-Book project.

Context:
- Frontend: Docusaurus (React, static)
- Backend: FastAPI (Python)
- Auth: Better Auth (email/password + Google OAuth)
- Database: Neon (Postgres)
- Architecture: API-first, decoupled frontend/backend
- Base path: /Physical-AI-Robotic-Book

Problems to solve:
1. Email/password login succeeds, but visiting /profile logs the user out and redirects to /login.
2. Google login shows "Welcome / login successful" but user is still treated as unauthenticated.
3. Signup successfully creates user in DB, but onboarding popup never appears and user remains unauthenticated.

Specification must include:
- Root-cause analysis for session loss, auth state mismatch, cookies, redirects, and token handling.
- Clear auth flow diagrams for: Email/password login, Google OAuth login, Signup → onboarding → authenticated session
- Exact requirements for: Session persistence (cookies / headers / storage), Frontend auth state management, Backend auth validation & middleware, Correct redirect behavior after login/signup
- Fixes for onboarding trigger logic and completion state.
- Alignment between Better Auth, frontend routes, and backend auth guards.
- Environment & config requirements (CORS, cookie flags, basePath, OAuth redirect URIs).

Testing:
- Write complete test cases: Unit tests (auth logic), Integration tests (frontend ↔ backend), E2E tests (login, signup, onboarding, profile access)
- Include failure cases and regression tests to prevent re-login/logout bugs.

Constraints:
- Do NOT rewrite the entire auth system.
- Use Better Auth correctly (Consider context7 better-auth MCP or better-auth MCP for latest documentation).
- Keep changes minimal, explicit, and production-safe.

Output:
- spec.md style document
- Clear acceptance criteria for each issue
- Test scenarios mapped to each fix

## Response snapshot

Successfully created comprehensive specification document for authentication session persistence fix.

**Key deliverables:**
1. **Root Cause Analysis**: Identified 3 primary failure modes:
   - Cookie domain/SameSite misconfiguration causing cross-origin cookie transmission failures
   - OAuth token exchange race condition preventing session establishment
   - Missing onboarding modal trigger logic

2. **User Stories**: 5 prioritized user stories (3x P1, 2x P2) with independent acceptance scenarios:
   - Email/password authentication flow
   - Google OAuth authentication flow
   - Onboarding completion after signup
   - Cross-origin session persistence
   - Session recovery after page reload

3. **Functional Requirements**: 39 testable requirements across 6 categories:
   - Session management (FR-001 to FR-008)
   - Email/password auth (FR-009 to FR-013)
   - OAuth auth (FR-014 to FR-020)
   - Onboarding flow (FR-021 to FR-025)
   - Frontend auth state (FR-026 to FR-030)
   - Session validation (FR-031 to FR-034)
   - Configuration (FR-035 to FR-039)

4. **Success Criteria**: 10 measurable, technology-agnostic outcomes including 100% test pass rates, zero support tickets, and correct cookie attributes

5. **Testing Strategy**: Comprehensive test plan covering:
   - Unit tests (backend session service, auth routes, OAuth routes, config)
   - Unit tests (frontend auth API, auth hook, OAuth callback)
   - Integration tests (login flow, OAuth flow, onboarding, session persistence, cross-origin)
   - E2E tests (complete auth journey, OAuth journey, logout, session expiry)
   - Regression tests (re-login, multiple tabs, OAuth tampering, cookie blocking)

6. **Quality Validation**: Created requirements checklist confirming:
   - Zero implementation details in requirements
   - All requirements testable and unambiguous
   - Success criteria are technology-agnostic and measurable
   - Edge cases identified
   - Scope clearly bounded
   - No [NEEDS CLARIFICATION] markers

**Technical Analysis:**
- Analyzed Better Auth documentation for session management, cookies, and OAuth best practices
- Reviewed existing FastAPI backend code (auth routes, OAuth routes, session service, config)
- Reviewed existing React frontend code (auth hook, auth API, OAuth callback handler)
- Identified specific code locations contributing to each failure mode

**Specification Quality:**
- All mandatory sections complete (User Scenarios, Requirements, Success Criteria, Testing)
- 7 edge cases documented
- 6 risks identified with mitigations
- 6 assumptions documented
- 9 out-of-scope items explicitly excluded
- Implementation notes provided as guidance (separated from requirements)

## Outcome

- ✅ Impact: Comprehensive specification ready for planning phase. Addresses all 3 authentication failure modes with clear, testable requirements.
- 🧪 Tests: Specification includes detailed testing strategy (unit, integration, E2E, regression) with specific test cases mapped to each requirement.
- 📁 Files: Created 2 files: `specs/001-fix-auth/spec.md` (full specification), `specs/001-fix-auth/checklists/requirements.md` (quality validation)
- 🔁 Next prompts: Run `/sp.clarify` if stakeholder input needed, then `/sp.plan` to create technical implementation plan, then `/sp.tasks` to generate task list
- 🧠 Reflection: Better Auth MCP search provided valuable insights into cookie cache strategies, session management patterns, and OAuth flow best practices. Root cause analysis grounded in actual code inspection (specific file:line references) ensures fixes target actual problems.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification workflow completed successfully.
- Graders run and results (PASS/FAIL): Requirements checklist validation PASSED (all 14 criteria met)
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Consider adding ADR suggestion for "Cross-Origin Cookie Strategy" decision (SameSite=Lax vs None based on environment)
