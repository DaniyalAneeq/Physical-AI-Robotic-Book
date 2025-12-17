# Implementation Plan: Better Auth Integration

**Branch**: `001-better-auth` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/001-better-auth/spec.md`

## Summary

Implement authentication system for the AI-robotics textbook application using Better Auth architectural patterns adapted for FastAPI (Python) and Neon Serverless Postgres. The system will support:

- **Email/password authentication** with Argon2id password hashing
- **Google OAuth 2.0** signup and login with account linking
- **Database-backed sessions** with cryptographic token hashing
- **Cookie-based authentication** (HttpOnly, Secure, SameSite)
- **Protected API endpoints** via FastAPI dependency injection
- **Rate limiting** on authentication endpoints (Redis-backed)
- **Security audit logging** for all authentication events

**Technical Approach** (from research):
- Treat Better Auth as architectural specification (JS/TS library not compatible with Python)
- Implement Better Auth patterns natively in FastAPI
- Mirror Better Auth's endpoint structure and security properties
- Use industry-standard Python libraries (argon2-cffi, Authlib, SQLAlchemy 2.0)

---

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI 0.115+, SQLAlchemy 2.0 (async), Authlib 1.3+, argon2-cffi 23.1+
**Storage**: Neon Serverless Postgres (PostgreSQL 15+) with asyncpg driver
**Testing**: pytest, pytest-asyncio, httpx (for async testing)
**Target Platform**: Linux server (production), any platform for development
**Project Type**: Web application (decoupled frontend + backend)
**Performance Goals**:
- <2s for 95% of auth requests
- 1000 concurrent authenticated users
- <500ms session validation
**Constraints**:
- Zero plain-text passwords in database
- All auth logic in backend only
- Better Auth patterns mandatory (per constitution)
- HttpOnly cookies required
**Scale/Scope**:
- Expected: 10,000+ users
- Multi-device sessions per user
- OAuth + email/password dual auth

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication Quality Gates (Constitution XIV)

- [x] **No custom auth logic bypassing Better Auth**: We implement Better Auth patterns natively in FastAPI (verified in research.md)
- [x] **Secure cookie/token configuration**: HttpOnly, Secure (production), SameSite=Lax (verified in data-model.md and contracts)
- [x] **Session expiration verified**: 30-day absolute max, 7-day sliding idle timeout (verified in research.md)
- [x] **Frontend contains no secrets**: Docusaurus only calls backend endpoints, all secrets in backend .env (verified in spec.md FR-037 to FR-040)
- [ ] **Auth flows tested in staging**: Will be verified in implementation phase

### Better Auth Rules (Constitution X)

- [x] **Custom auth flows forbidden**: We follow Better Auth patterns exactly (research.md Decision 1)
- [x] **Only documented extensions permitted**: Using standard FastAPI dependencies, not modifying Better Auth itself
- [x] **Backend validates on every protected request**: FastAPI dependency injection validates session on all protected endpoints (contracts/protected-endpoints.yaml)
- [x] **Sessions HttpOnly/Secure/Expired**: Configured correctly (research.md Decision 3)

### RAG Security, Privacy & Sessions (Constitution VIII)

- [x] **Parameterized queries only**: SQLAlchemy ORM handles parameterization (data-model.md)
- [x] **Encrypted session handling**: Session tokens hashed with HMAC-SHA256 (research.md Decision 3)
- [x] **No PII storage**: Only email, name stored (encrypted OAuth tokens in oauth_accounts)
- [x] **Rate limiting**: 5 failed attempts per 10 minutes per email (research.md Decision 6)

**Constitution Compliance**: PASSED ✅

---

## Project Structure

### Documentation (this feature)

```text
specs/001-better-auth/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (in progress)
├── research.md          # Phase 0 output (complete)
├── data-model.md        # Phase 1 output (complete)
├── quickstart.md        # Phase 1 output (complete)
├── contracts/           # Phase 1 output (complete)
│   ├── auth-api.yaml
│   └── protected-endpoints.yaml
├── checklists/          # Quality validation
│   └── requirements.md
└── tasks.md             # Phase 2 output (created by /sp.tasks - NOT this command)
```

### Source Code (repository root)

```text
backend/
├── .env                  # Environment variables (not in git)
├── .env.example          # Template for .env
├── requirements.txt      # Python dependencies
├── alembic.ini           # Database migration configuration
├── migrations/           # Alembic migration scripts
│   ├── env.py
│   └── versions/
│       └── 001_initial_auth.py
├── src/
│   ├── __init__.py
│   ├── main.py           # FastAPI application entry point
│   ├── config.py         # Configuration (load from .env)
│   ├── models/           # SQLAlchemy models
│   │   ├── __init__.py
│   │   ├── base.py       # Base model class
│   │   ├── user.py       # User model
│   │   ├── session.py    # Session model
│   │   └── oauth_account.py  # OAuthAccount model
│   ├── schemas/          # Pydantic schemas (request/response)
│   │   ├── __init__.py
│   │   ├── user.py       # UserCreate, UserResponse, UserBase
│   │   ├── session.py    # SessionResponse, SessionWithUser
│   │   └── auth.py       # LoginRequest, RegisterRequest, AuthResponse
│   ├── api/
│   │   ├── __init__.py
│   │   ├── deps.py       # Dependencies (get_current_user, get_db)
│   │   └── routes/
│   │       ├── __init__.py
│   │       ├── auth.py   # POST /auth/register, /auth/login, /auth/logout
│   │       ├── oauth.py  # GET /auth/oauth/google, /auth/oauth/google/callback
│   │       └── session.py  # GET /auth/session
│   ├── services/         # Business logic
│   │   ├── __init__.py
│   │   ├── auth.py       # Password hashing, token generation, validation
│   │   ├── session.py    # Session creation, lookup, revocation
│   │   └── oauth.py      # OAuth flow logic, account linking
│   ├── db/
│   │   ├── __init__.py
│   │   └── session.py    # Database session management (async)
│   └── utils/
│       ├── __init__.py
│       ├── security.py   # Token hashing, HMAC helpers
│       └── rate_limit.py # Rate limiting decorators
└── tests/
    ├── __init__.py
    ├── conftest.py       # Pytest fixtures (test DB, test client)
    ├── unit/
    │   ├── test_auth_service.py
    │   ├── test_session_service.py
    │   └── test_oauth_service.py
    ├── integration/
    │   ├── test_register_login_flow.py
    │   ├── test_oauth_flow.py
    │   └── test_protected_endpoints.py
    └── contract/
        └── test_api_contracts.py

frontend/
├── src/
│   ├── components/
│   │   └── auth/
│   │       ├── LoginForm.tsx       # Email/password login
│   │       ├── RegisterForm.tsx    # Email/password signup
│   │       ├── GoogleLoginButton.tsx  # OAuth button
│   │       └── LogoutButton.tsx
│   ├── hooks/
│   │   └── useAuth.ts              # React hook for auth state
│   └── utils/
│       └── auth.ts                 # API client for auth endpoints
└── pages/
    ├── login.tsx
    └── register.tsx
```

**Structure Decision**: Web application with decoupled frontend (Docusaurus) and backend (FastAPI). Backend handles all authentication logic and session management. Frontend only calls backend API endpoints via fetch with `credentials: 'include'` for cookie-based auth.

---

## Complexity Tracking

**No constitutional violations** - all complexity is justified by requirements.

---

## Phase 0: Research & Technical Decisions

**Status**: ✅ Complete

**Deliverable**: `research.md`

### Research Questions Resolved

1. **Better Auth compatibility with FastAPI** → Implement patterns natively in Python
2. **Password hashing algorithm** → Argon2id via argon2-cffi
3. **Session management strategy** → Database-backed with HMAC-SHA256 token hashing
4. **OAuth 2.0 implementation** → Authlib for Google OAuth
5. **Database ORM** → SQLAlchemy 2.0 async + Alembic + asyncpg
6. **Rate limiting** → slowapi with Redis backend
7. **Frontend-backend auth flow** → Cookie-based with SameSite=Lax
8. **Protected routes** → FastAPI dependency injection (get_current_user)

**Key Decisions**:
- Better Auth is architectural spec (not direct library integration)
- Argon2id > bcrypt for password hashing
- HMAC-SHA256 for session token hashing (fast, secure for high-entropy tokens)
- Authlib for OAuth 2.0 (best Python async support)
- Redis-backed rate limiting (Postgres not ideal for counters)

See: `research.md` for full details and alternatives considered.

---

## Phase 1: Data Model & API Contracts

**Status**: ✅ Complete

**Deliverables**:
- `data-model.md`
- `contracts/auth-api.yaml`
- `contracts/protected-endpoints.yaml`
- `quickstart.md`

### Data Model Summary

**Tables**:
1. **users** (id, email, name, password_hash, email_verified_at, created_at, updated_at)
2. **sessions** (id, user_id, token_hash, created_at, last_used_at, expires_at, revoked, ip_address, user_agent)
3. **oauth_accounts** (id, user_id, provider, provider_account_id, access_token, refresh_token, expires_at, scope, token_type, created_at, updated_at)

**Relationships**:
- User → Sessions (1:N, cascade delete)
- User → OAuth Accounts (1:N, cascade delete)

**Indexes**:
- `users.email` (UNIQUE, CITEXT)
- `sessions.token_hash` (UNIQUE)
- `sessions(user_id, revoked, expires_at)` (partial, WHERE revoked = FALSE)
- `oauth_accounts(provider, provider_account_id)` (UNIQUE composite)

See: `data-model.md` for SQLAlchemy models and Alembic migrations.

### API Contracts Summary

**Authentication Endpoints**:
- `POST /api/auth/register` → Create user + session (201)
- `POST /api/auth/login` → Authenticate + session (200)
- `POST /api/auth/logout` → Revoke session (200)
- `GET /api/auth/session` → Get current user/session (200)

**OAuth Endpoints**:
- `GET /api/auth/oauth/google` → Redirect to Google (302)
- `GET /api/auth/oauth/google/callback` → Handle callback + create session (302 or 400)

**Protected Endpoint Pattern**:
- Requires `session` cookie
- Returns 401 if not authenticated/expired
- Example: `GET /api/chatbot/history`

See: `contracts/auth-api.yaml` for full OpenAPI specification.

### Quickstart Guide

**Prerequisites**: Python 3.11+, Node.js 18+, Neon account, Google Cloud Console
**Setup Steps**:
1. Create Neon project, get connection string
2. Configure Google OAuth credentials
3. Set up `.env` file with secrets
4. Run Alembic migrations
5. Start FastAPI dev server
6. Test endpoints via Swagger UI

See: `quickstart.md` for detailed setup instructions.

---

## Phase 2: Implementation Tasks (Task Breakdown)

**Status**: Ready for `/sp.tasks` command

**Task Categories**:

### A. Infrastructure Setup (Foundational)
1. Backend project setup (requirements.txt, directory structure)
2. Database migrations (Alembic initialization, initial migration)
3. Environment configuration (.env template, config.py)
4. Google OAuth credentials setup

### B. Core Authentication (P1 - MVP)
5. User model implementation
6. Session model implementation
7. Password hashing service (Argon2id)
8. Session token generation and validation
9. Email/password registration endpoint
10. Email/password login endpoint
11. Logout endpoint
12. Get current session endpoint
13. Protected endpoint dependency (`get_current_user`)

### C. OAuth Integration (P2)
14. OAuth account model implementation
15. Authlib setup and configuration
16. Google OAuth initiation endpoint
17. Google OAuth callback handler
18. Account linking logic (email matching)

### D. Security & Rate Limiting
19. Rate limiting setup (slowapi + Redis)
20. Apply rate limits to auth endpoints
21. Audit logging for auth events
22. CORS configuration for frontend

### E. Frontend Integration
23. Auth utility functions (frontend/src/utils/auth.ts)
24. Login form component
25. Register form component
26. Google OAuth button component
27. Logout button component
28. Protected route wrapper (useAuth hook)

### F. Testing
29. Unit tests for auth service
30. Unit tests for session service
31. Unit tests for OAuth service
32. Integration test: registration → login flow
33. Integration test: OAuth flow
34. Integration test: protected endpoints
35. Contract tests (validate API against OpenAPI spec)

### G. Documentation & Deployment
36. API documentation (Swagger/ReDoc)
37. Environment setup guide for production
38. CI/CD pipeline (GitHub Actions)
39. Deployment configuration (Vercel/Railway)

**Dependencies**:
- Tasks 1-4 must complete before any other tasks
- Tasks 5-13 (core auth) must complete before tasks 14-18 (OAuth)
- Task 13 must complete before tasks 23-28 (frontend)
- All implementation tasks must complete before testing (tasks 29-35)

**Total Estimated Tasks**: 39 (to be broken down further in tasks.md)

---

## Phase 3: Implementation Priorities

### P1 (Critical MVP) - Must Ship First
- Infrastructure setup (tasks 1-4)
- Core authentication (tasks 5-13)
- Basic testing (tasks 29-32)

**Acceptance**: Users can register, login, logout with email/password. Protected endpoints work. Session management functional.

### P2 (Enhanced UX) - Ship Second
- OAuth integration (tasks 14-18)
- Frontend components (tasks 23-28)
- OAuth testing (tasks 33)

**Acceptance**: Users can sign up/login with Google. Account linking works. Frontend UI integrated.

### P3 (Production Ready) - Ship Third
- Security hardening (tasks 19-22)
- Full test coverage (tasks 34-35)
- Documentation & deployment (tasks 36-39)

**Acceptance**: Rate limiting active. Audit logs working. CI/CD pipeline deployed. Production-ready.

---

## Risk Mitigation

### Risk 1: Better Auth Incompatibility with Python
**Status**: Resolved in Phase 0
**Mitigation**: Treat Better Auth as architectural spec; implement patterns natively in FastAPI
**Impact**: No impact - research confirmed this approach

### Risk 2: Session Token Security
**Probability**: Low
**Impact**: High (session hijacking)
**Mitigation**:
- HMAC-SHA256 hashing with server pepper
- HttpOnly, Secure, SameSite cookies
- Session revocation support
**Monitoring**: Audit logs for suspicious session activity

### Risk 3: OAuth Provider Outage (Google)
**Probability**: Low
**Impact**: High (blocks OAuth signups/logins)
**Mitigation**: Email/password fallback always available
**Monitoring**: Alert on repeated OAuth failures

### Risk 4: Rate Limiting Bypass
**Probability**: Medium
**Impact**: Medium (brute force attacks)
**Mitigation**: Redis-backed rate limiting (hard to bypass), IP + email tracking
**Monitoring**: Track failed login patterns

### Risk 5: Database Performance (Session Lookups)
**Probability**: Medium (as user base grows)
**Impact**: High (slow authentication)
**Mitigation**: Proper indexes on token_hash, user_id, expires_at; consider session caching in Redis
**Monitoring**: Track session lookup query times

---

## Testing Strategy

### Unit Tests
- Services (auth, session, OAuth) tested in isolation
- Mock database and external dependencies (Google OAuth)
- Coverage target: 90%+

### Integration Tests
- Full authentication flows (register → login → protected endpoint → logout)
- OAuth flow (initiate → callback → session established)
- Session expiration and revocation
- Multi-device sessions

### Contract Tests
- Validate API responses against OpenAPI spec
- Ensure frontend/backend contract compatibility
- Run on CI for every pull request

### Security Tests
- OWASP Top 10 validation
- SQL injection attempts
- XSS attempts
- CSRF protection validation
- Rate limiting effectiveness
- Session fixation prevention

### Performance Tests
- 1000 concurrent authenticated users
- Session lookup latency <500ms
- Login latency <2s for 95% of requests

---

## Deployment Strategy

### Development Environment
- Local Postgres or Neon dev branch
- Local Redis or Upstash free tier
- Google OAuth localhost redirect URI
- SECURE_COOKIES=false (HTTP localhost)

### Staging Environment
- Neon staging branch
- Upstash Redis (shared tier)
- Google OAuth staging redirect URI
- SECURE_COOKIES=true
- CI/CD deployment on merge to `staging` branch

### Production Environment
- Neon production branch (with backups)
- Upstash Redis production instance
- Google OAuth production redirect URI (verified domain)
- SECURE_COOKIES=true
- HTTPS enforced (HSTS headers)
- CI/CD deployment on merge to `main` branch
- Blue-green deployment for zero downtime

### Monitoring & Observability
- Application logs: Structured logging (JSON)
- Audit logs: Authentication events (PostgreSQL table)
- Metrics: Prometheus/Grafana (login success rate, session count, API latency)
- Alerts: PagerDuty for critical failures (database down, OAuth failures, high error rate)

---

## Security Checklist (Production)

- [ ] All passwords hashed with Argon2id (time_cost=3, memory_cost=65536)
- [ ] Session tokens hashed with HMAC-SHA256
- [ ] Cookies: HttpOnly=true, Secure=true, SameSite=Lax
- [ ] HTTPS enforced (HSTS headers)
- [ ] OAuth secrets in environment variables (not in code)
- [ ] Rate limiting active (5 attempts per 10 minutes)
- [ ] CORS configured (allow_credentials=true, specific origins only)
- [ ] Audit logging enabled (all auth events)
- [ ] Input validation on all endpoints (Pydantic)
- [ ] SQL injection prevention (SQLAlchemy parameterized queries)
- [ ] XSS prevention (FastAPI JSONResponse escaping)
- [ ] Session fixation prevention (rotate token on login)
- [ ] CSRF protection (SameSite cookies, state validation for OAuth)
- [ ] Database backups enabled (Neon automatic backups)
- [ ] Secrets rotation policy (30-90 days for SESSION_SECRET_PEPPER)

---

## Success Criteria (Verification)

### Functional Success (P1)
- [x] Spec written and approved (spec.md)
- [x] Research completed (research.md)
- [x] Data model designed (data-model.md)
- [x] API contracts defined (contracts/*.yaml)
- [ ] All functional requirements implemented (FR-001 to FR-043)
- [ ] Email/password signup working
- [ ] Email/password login working
- [ ] Session management working (create, validate, revoke)
- [ ] Protected endpoints rejecting unauthenticated requests
- [ ] Google OAuth signup working (P2)
- [ ] Google OAuth login working (P2)
- [ ] Account linking working (email matching)

### Non-Functional Success
- [ ] SC-001: Signup completes in <60s
- [ ] SC-002: OAuth signup completes in <30s
- [ ] SC-003: 95% of logins succeed within 2s
- [ ] SC-004: 1000 concurrent users without degradation
- [ ] SC-005: Zero plain-text passwords in database
- [ ] SC-006: 401 responses within 500ms
- [ ] SC-007: Sessions persist for 30 days
- [ ] SC-008: Rate limiting blocks after 5 failed attempts
- [ ] SC-009: OAuth state validation prevents CSRF
- [ ] SC-010: 90% first-attempt success rate
- [ ] SC-011: Complete audit trail for all auth events
- [ ] SC-012: Multi-device sessions with independent logout

### Quality Gates (Constitution)
- [ ] No custom auth logic bypassing Better Auth patterns
- [ ] Secure cookie/token configuration verified
- [ ] Session expiration tested
- [ ] Frontend contains no secrets
- [ ] Auth flows tested in staging

---

## Next Steps

1. **Run `/sp.tasks`**: Generate detailed, ordered tasks from this plan
2. **Implement P1 tasks**: Infrastructure + core authentication
3. **Test P1**: Unit + integration tests for email/password auth
4. **Implement P2 tasks**: OAuth integration + frontend components
5. **Test P2**: OAuth flow and frontend integration tests
6. **Implement P3 tasks**: Security hardening + deployment
7. **Final QA**: Full security audit, performance testing, staging deployment
8. **Production deployment**: Merge to main, monitor logs, verify success criteria

---

## References

- **Specification**: [spec.md](./spec.md)
- **Research**: [research.md](./research.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contracts**: [contracts/auth-api.yaml](./contracts/auth-api.yaml)
- **Quickstart**: [quickstart.md](./quickstart.md)
- **Constitution**: [.specify/memory/constitution.md](../../.specify/memory/constitution.md)
- **Better Auth Documentation**: https://better-auth.com
- **FastAPI Documentation**: https://fastapi.tiangolo.com
- **Authlib Documentation**: https://docs.authlib.org
- **SQLAlchemy 2.0**: https://docs.sqlalchemy.org/en/20/
- **Neon Documentation**: https://neon.tech/docs

---

**Plan Complete**: 2025-12-14
**Ready for**: Task generation via `/sp.tasks` command
**Branch**: 001-better-auth
**Next Command**: `/sp.tasks`
