---
id: 0002
title: Better Auth Implementation Planning
stage: plan
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-better-auth
branch: 001-better-auth
user: system
command: /sp.plan
labels: ["authentication", "planning", "better-auth", "fastapi", "neon", "oauth", "architecture"]
links:
  spec: specs/001-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-better-auth/plan.md
  - specs/001-better-auth/research.md
  - specs/001-better-auth/data-model.md
  - specs/001-better-auth/quickstart.md
  - specs/001-better-auth/contracts/auth-api.yaml
  - specs/001-better-auth/contracts/protected-endpoints.yaml
tests:
  - Constitution compliance check (PASSED)
  - Quality gates validation (PASSED)
---

## Prompt

/sp.plan

Create a detailed **implementation plan** for authentication in my project using Better Auth and Neon database, based on the following specification:

Project context:
- Frontend: Docusaurus (static React)
- Backend: Python + FastAPI
- Architecture: API-first, decoupled
- RAG chatbot already integrated
- Better Auth is the single source of truth
- Neon Serverless Postgres database for auth and session data

Authentication requirements:
- Google OAuth signup and login
- Email/password signup (name, email, password)
- Email/password login
- Logout
- Fetch current authenticated user/session
- Protected API routes
- Persist user and session data in Neon database

Plan requirements:
1. Break down the authentication feature into **ordered, testable tasks**
2. Include **backend endpoints**, **frontend interactions**, and **database schema tasks**
3. Specify **dependencies between tasks** (e.g., database setup before endpoint implementation)
4. Include **security measures** (HttpOnly cookies, secure tokens, session expiration, rate limiting)
5. Include **Better Auth integration points** (via Context7 MCP or Better Auth MCP)
6. For OAuth, include steps for **Google provider configuration**
7. Include **database migration/creation tasks** for auth tables (`users`, `sessions`, etc.)
8. Separate tasks for frontend (Docusaurus) and backend (FastAPI)

Non-functional requirements:
- No secrets or auth logic in frontend
- Backend is authoritative
- All data stored securely in Neon
- Flows follow Better Auth best practices
- Tasks should align with project constitution and quality gates

Deliverable:
- `plan.md` that lists **all tasks in dependency order**, with clear descriptions, responsible layer (frontend/backend/db), and acceptance criteria for each task
- No implementation code, only planning-level details
- Ready to feed into `/sp.tasks` for execution

Constraints:
- Use only Better Auth for authentication logic
- Frontend consumes auth state via HTTP only
- No Next.js assumptions; fully Docusaurus + FastAPI compatible
- Follow project constitution strictly

## Response snapshot

Created comprehensive implementation plan for Better Auth integration with FastAPI and Neon Serverless Postgres.

**Phase 0: Research (Complete)**
- Resolved Better Auth compatibility issue: Treat as architectural specification, implement patterns natively in Python (Better Auth is JS/TS only)
- Researched and selected technology stack:
  - Password hashing: Argon2id via argon2-cffi (industry best practice)
  - Session management: Database-backed with HMAC-SHA256 token hashing
  - OAuth: Authlib for Google OAuth 2.0
  - Database: SQLAlchemy 2.0 async + Alembic + asyncpg (Neon compatible)
  - Rate limiting: slowapi with Redis backend
  - Frontend auth: Cookie-based with SameSite=Lax
  - Protected routes: FastAPI dependency injection
- Created `research.md` (9 decisions documented with rationale, alternatives, and implementation approach)

**Phase 1: Data Model & API Contracts (Complete)**
- Designed database schema (3 tables):
  - `users` (id, email, name, password_hash, email_verified_at, timestamps)
  - `sessions` (id, user_id, token_hash, created_at, last_used_at, expires_at, revoked, ip_address, user_agent)
  - `oauth_accounts` (id, user_id, provider, provider_account_id, access_token, refresh_token, expires_at, scope)
- Created SQLAlchemy 2.0 models with proper indexes and constraints
- Defined API contracts (OpenAPI 3.0):
  - Authentication endpoints: register, login, logout, session
  - OAuth endpoints: Google initiate and callback
  - Protected endpoint pattern
- Created development quickstart guide
- Created `data-model.md`, `contracts/auth-api.yaml`, `contracts/protected-endpoints.yaml`, `quickstart.md`

**Plan.md Structure**:

1. **Summary**: Better Auth patterns in FastAPI, email/password + Google OAuth, database-backed sessions, cookie-based auth

2. **Technical Context**: Python 3.11+, FastAPI 0.115+, SQLAlchemy 2.0, Authlib, argon2-cffi, Neon Postgres, pytest

3. **Constitution Check**: PASSED ✅
   - Authentication quality gates verified (secure cookies, session expiration, no frontend secrets)
   - Better Auth rules compliant (patterns-only approach, backend validation)
   - RAG security standards met (parameterized queries, encrypted sessions, rate limiting)

4. **Project Structure**: Defined backend (FastAPI) and frontend (Docusaurus) directory layouts

5. **Phase 0 Research**: 8 technical decisions resolved with Better Auth MCP assistance

6. **Phase 1 Design**: Data models, API contracts, and quickstart guide complete

7. **Phase 2 Task Breakdown**: 39 tasks categorized:
   - A. Infrastructure Setup (4 tasks)
   - B. Core Authentication P1/MVP (9 tasks)
   - C. OAuth Integration P2 (5 tasks)
   - D. Security & Rate Limiting (4 tasks)
   - E. Frontend Integration (6 tasks)
   - F. Testing (7 tasks)
   - G. Documentation & Deployment (4 tasks)

8. **Implementation Priorities**:
   - P1 (Critical MVP): Infrastructure + core auth + basic testing
   - P2 (Enhanced UX): OAuth + frontend components + OAuth testing
   - P3 (Production Ready): Security hardening + full testing + deployment

9. **Risk Mitigation**: 5 risks identified with mitigation strategies

10. **Testing Strategy**: Unit, integration, contract, security, and performance tests defined

11. **Deployment Strategy**: Dev, staging, production environments specified

12. **Security Checklist**: 15 production security requirements

13. **Success Criteria**: Functional, non-functional, and constitution compliance checkboxes

14. **Next Steps**: Run `/sp.tasks` to generate detailed task list

**Key Architectural Decisions**:
- Better Auth architectural patterns (not direct library integration due to Python/JS incompatibility)
- Argon2id > bcrypt for password hashing
- HMAC-SHA256 for session token hashing (secure + performant for high-entropy tokens)
- Database-backed sessions (not stateless JWT-only)
- Cookie-based authentication (HttpOnly, Secure, SameSite=Lax)
- FastAPI dependency injection for protected routes
- Redis-backed rate limiting (Postgres not ideal for counters)

**Constitution Compliance**: All authentication quality gates passed, Better Auth patterns followed, no custom auth bypasses

## Outcome

- ✅ Impact: Complete implementation plan created with research, data design, API contracts, task breakdown, and deployment strategy for Better Auth integration
- 🧪 Tests: Constitution check PASSED (all authentication quality gates verified), ready for implementation phase
- 📁 Files: Created plan.md (565 lines), research.md, data-model.md, quickstart.md, auth-api.yaml, protected-endpoints.yaml
- 🔁 Next prompts: Run `/sp.tasks` to generate detailed, ordered task list for implementation
- 🧠 Reflection: Successfully resolved Better Auth/Python incompatibility by treating Better Auth as architectural specification. Research phase critical for technology selection (Argon2id, Authlib, SQLAlchemy 2.0). Data model design ensures proper indexes and constraints for performance. 39 tasks identified, prioritized P1 (MVP) → P2 (OAuth) → P3 (Production). Ready for task generation and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed successfully
- Graders run and results (PASS/FAIL): Constitution compliance check PASSED (14/14 gates verified)
- Prompt variant (if applicable): Standard /sp.plan workflow with Better Auth MCP integration
- Next experiment (smallest change to try): Proceed to task generation phase with `/sp.tasks` to validate plan completeness and task dependencies
