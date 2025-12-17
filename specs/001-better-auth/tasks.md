# Implementation Tasks: Better Auth Integration

**Feature Branch**: `001-better-auth`
**Created**: 2025-12-14
**Source**: Generated from `/sp.tasks` based on [spec.md](./spec.md) and [plan.md](./plan.md)

---

## Task Organization

Tasks are organized by **user story** to enable independent implementation and testing. Each user story can be completed and validated independently, allowing parallel development and incremental delivery.

**Task Format**:
```
- [ ] [TaskID] [Priority] [Story] Description with file path
```

**Priority Levels**:
- **P1**: Critical MVP (must ship first)
- **P2**: Enhanced UX (ship second)
- **P3**: Production ready (ship third)

**User Story Labels**:
- **SETUP**: Foundational infrastructure
- **US1**: Email/Password Account Creation
- **US2**: Email/Password Login
- **US3**: Google OAuth Signup
- **US4**: Google OAuth Login
- **US5**: Session Management and Logout
- **US6**: Protected Resource Access

---

## User Stories Roadmap

| Story | Priority | Description | Independent Test |
|-------|----------|-------------|------------------|
| US1 | P1 | Email/Password Account Creation | Submit signup form → verify account created → verify can login |
| US2 | P1 | Email/Password Login | Create test account → logout → login with credentials → verify session |
| US3 | P2 | Google OAuth Signup | Click "Sign up with Google" → complete OAuth → verify account created |
| US4 | P2 | Google OAuth Login | OAuth signup → logout → OAuth login → verify same account |
| US5 | P1 | Session Management and Logout | Login → verify session persists → logout → verify session terminated |
| US6 | P1 | Protected Resource Access | With session → access protected endpoint succeeds; Without session → returns 401 |

---

## Phase 0: Setup (SETUP)

**Purpose**: Establish foundational infrastructure required by all user stories.

**Dependencies**: None (can start immediately)

**Acceptance**: Backend runs, database connected, migrations executable, environment configured, Google OAuth credentials ready.

### Tasks

- [ ] [TASK-001] [P1] [SETUP] Create backend project structure at `backend/` with directory layout from plan.md:285
- [ ] [TASK-002] [P1] [SETUP] Create `backend/requirements.txt` with all dependencies from quickstart.md:46-85
- [ ] [TASK-003] [P1] [SETUP] Create `backend/.env.example` template with all required variables from research.md:429-456
- [ ] [TASK-004] [P1] [SETUP] Initialize Alembic migrations: create `backend/alembic.ini` and `backend/migrations/env.py` from quickstart.md:198-246
- [ ] [TASK-005] [P1] [SETUP] Create FastAPI app entry point at `backend/src/main.py` with CORS configuration from research.md:282-292
- [ ] [TASK-006] [P1] [SETUP] Create database configuration at `backend/src/config.py` using Pydantic Settings from quickstart.md:114-144
- [ ] [TASK-007] [P1] [SETUP] Create async database session manager at `backend/src/db/session.py` with SQLAlchemy 2.0 async from data-model.md:210-223
- [ ] [TASK-008] [P1] [SETUP] Obtain Google OAuth credentials from Google Cloud Console following quickstart.md:155-191
- [ ] [TASK-009] [P1] [SETUP] Create SQLAlchemy Base model at `backend/src/models/base.py` from data-model.md:210-223
- [ ] [TASK-010] [P1] [SETUP] Create Alembic migration `001_initial_auth.py` for users/sessions/oauth_accounts tables from data-model.md:518-586

**Parallel Execution**:
```
|| TASK-001, TASK-002, TASK-003, TASK-008 (independent setup tasks)
→ TASK-004 (requires TASK-001: directory structure)
→ TASK-009 (requires TASK-001: directory structure)
→ TASK-005, TASK-006, TASK-007 (requires TASK-001, TASK-002: project + dependencies)
→ TASK-010 (requires TASK-004, TASK-009: Alembic + Base model)
```

---

## Phase 1: User Story 1 - Email/Password Account Creation (US1) - P1

**Purpose**: Users can create accounts using name, email, and password.

**Dependencies**: Phase 0 (SETUP)

**Acceptance Scenarios**:
1. Valid name, email, password → account created + session established
2. Existing email → clear error "Email already registered"
3. Weak password (< 8 chars) → validation feedback
4. Invalid email format → validation feedback

**Independent Test**: Submit signup form with valid credentials → verify user created in database → verify can login immediately → verify session cookie set

### Tasks

- [ ] [TASK-011] [P1] [US1] Create User SQLAlchemy model at `backend/src/models/user.py` from data-model.md:229-285
- [ ] [TASK-012] [P1] [US1] Create Session SQLAlchemy model at `backend/src/models/session.py` from data-model.md:290-354
- [ ] [TASK-013] [P1] [US1] Create User Pydantic schemas at `backend/src/schemas/user.py` (UserBase, UserCreate, UserResponse) from data-model.md:442-465
- [ ] [TASK-014] [P1] [US1] Create Session Pydantic schemas at `backend/src/schemas/session.py` (SessionResponse) from data-model.md:472-486
- [ ] [TASK-015] [P1] [US1] Create auth request/response schemas at `backend/src/schemas/auth.py` (RegisterRequest, AuthResponse) from data-model.md:492-508
- [ ] [TASK-016] [P1] [US1] Implement password hashing service at `backend/src/services/auth.py` using Argon2id from research.md:40-62
- [ ] [TASK-017] [P1] [US1] Implement session token generation at `backend/src/utils/security.py` using HMAC-SHA256 from research.md:76-105
- [ ] [TASK-018] [P1] [US1] Implement session creation service at `backend/src/services/session.py` with token generation and database persistence
- [ ] [TASK-019] [P1] [US1] Create registration endpoint POST `/api/auth/register` at `backend/src/api/routes/auth.py` implementing spec.md:506-563
- [ ] [TASK-020] [P1] [US1] Add email uniqueness validation to registration endpoint (check existing user before insert)
- [ ] [TASK-021] [P1] [US1] Add password strength validation to registration endpoint (min 8 chars, letters + numbers)
- [ ] [TASK-022] [P1] [US1] Set session cookie in registration endpoint response (HttpOnly, Secure, SameSite=Lax) from research.md:96-105
- [ ] [TASK-023] [P1] [US1] Write unit test for password hashing service at `backend/tests/unit/test_auth_service.py` (hash, verify, argon2id format)
- [ ] [TASK-024] [P1] [US1] Write unit test for session token generation at `backend/tests/unit/test_session_service.py` (randomness, hash uniqueness)
- [ ] [TASK-025] [P1] [US1] Write integration test for registration flow at `backend/tests/integration/test_register_login_flow.py` (valid signup → user created → session created → cookie set)
- [ ] [TASK-026] [P1] [US1] Write integration test for duplicate email at `backend/tests/integration/test_register_login_flow.py` (signup twice same email → 409 Conflict)
- [ ] [TASK-027] [P1] [US1] Write integration test for weak password at `backend/tests/integration/test_register_login_flow.py` (password < 8 chars → 400 Bad Request)

**Parallel Execution**:
```
|| TASK-011, TASK-012 (SQLAlchemy models, independent)
→ || TASK-013, TASK-014, TASK-015 (Pydantic schemas, independent after models)
→ || TASK-016, TASK-017 (password hashing + token generation, independent services)
→ TASK-018 (session service, depends on TASK-017: token generation)
→ TASK-019 (registration endpoint, depends on TASK-011, TASK-012, TASK-013, TASK-015, TASK-016, TASK-018)
→ || TASK-020, TASK-021, TASK-022 (endpoint enhancements, can parallelize)
→ || TASK-023, TASK-024, TASK-025, TASK-026, TASK-027 (all tests, independent)
```

---

## Phase 2: User Story 2 - Email/Password Login (US2) - P1

**Purpose**: Users can log back into their accounts using email and password.

**Dependencies**: US1 (uses same User model, password hashing, session creation)

**Acceptance Scenarios**:
1. Correct email + password → authenticated + redirected with session
2. Incorrect password → generic error "Invalid email or password"
3. Unregistered email → generic error "Invalid email or password"
4. 5 failed attempts in 10 minutes → rate limit error with wait time

**Independent Test**: Create test user via US1 → logout → login with same credentials → verify session established → verify can access protected resources

### Tasks

- [ ] [TASK-028] [P1] [US2] Implement password verification in auth service at `backend/src/services/auth.py` (constant-time comparison) from research.md:46-62
- [ ] [TASK-029] [P1] [US2] Create login endpoint POST `/api/auth/login` at `backend/src/api/routes/auth.py` implementing spec.md:566-610
- [ ] [TASK-030] [P1] [US2] Add user lookup by email (case-insensitive) to login endpoint
- [ ] [TASK-031] [P1] [US2] Add session creation and cookie setting to login endpoint (reuse US1 session logic)
- [ ] [TASK-032] [P1] [US2] Implement generic error message for failed login (prevent email enumeration) per spec.md:600-604
- [ ] [TASK-033] [P3] [US2] Add rate limiting to login endpoint (5 attempts per 10 minutes per email) at `backend/src/utils/rate_limit.py` from research.md:221-255
- [ ] [TASK-034] [P3] [US2] Add audit logging for login attempts at `backend/src/services/logging.py` (timestamp, email, IP, result) from research.md:416-420
- [ ] [TASK-035] [P1] [US2] Write unit test for password verification at `backend/tests/unit/test_auth_service.py` (correct password → True, wrong password → False)
- [ ] [TASK-036] [P1] [US2] Write integration test for successful login at `backend/tests/integration/test_register_login_flow.py` (signup → logout → login → session valid)
- [ ] [TASK-037] [P1] [US2] Write integration test for failed login (wrong password) at `backend/tests/integration/test_register_login_flow.py` (login with wrong password → 401 Unauthorized)
- [ ] [TASK-038] [P3] [US2] Write integration test for rate limiting at `backend/tests/integration/test_register_login_flow.py` (6 failed logins → 429 Too Many Requests)

**Parallel Execution**:
```
TASK-028 (password verification, extends US1 auth service)
→ TASK-029 (login endpoint skeleton)
→ || TASK-030, TASK-031, TASK-032 (endpoint logic, can parallelize)
→ || TASK-033, TASK-034 (rate limiting + logging, P3 tasks, can defer or parallelize)
→ || TASK-035, TASK-036, TASK-037, TASK-038 (all tests, independent)
```

---

## Phase 3: User Story 5 - Session Management and Logout (US5) - P1

**Purpose**: Users can securely end sessions and sessions persist correctly.

**Dependencies**: US1 (session creation), US2 (login creates session)

**Acceptance Scenarios**:
1. Authenticated user clicks logout → session terminated + cookie cleared + redirected to login
2. Authenticated user closes browser → returns within 30 days → still logged in
3. Session expired → access protected resource → redirected to login with expiration message
4. Login from new device → independent session created (multi-device support)

**Independent Test**: Login → verify session persists across page refreshes → logout → verify session revoked in database → verify cannot access protected resources

### Tasks

- [ ] [TASK-039] [P1] [US5] Implement session validation in session service at `backend/src/services/session.py` (check revoked, expired, last_used_at update)
- [ ] [TASK-040] [P1] [US5] Implement session revocation in session service at `backend/src/services/session.py` (set revoked=TRUE in database)
- [ ] [TASK-041] [P1] [US5] Create logout endpoint POST `/api/auth/logout` at `backend/src/api/routes/auth.py` implementing spec.md:617-640
- [ ] [TASK-042] [P1] [US5] Add session cookie clearing to logout endpoint (Set-Cookie with Max-Age=0) from spec.md:631-632
- [ ] [TASK-043] [P1] [US5] Create session retrieval endpoint GET `/api/auth/session` at `backend/src/api/routes/session.py` implementing spec.md:685-716
- [ ] [TASK-044] [P1] [US5] Add sliding expiration logic to session validation (update last_used_at on each request) from research.md:395-397
- [ ] [TASK-045] [P1] [US5] Write unit test for session validation at `backend/tests/unit/test_session_service.py` (valid session → True, expired → False, revoked → False)
- [ ] [TASK-046] [P1] [US5] Write integration test for logout at `backend/tests/integration/test_register_login_flow.py` (login → logout → session revoked → cookie cleared)
- [ ] [TASK-047] [P1] [US5] Write integration test for session persistence at `backend/tests/integration/test_register_login_flow.py` (login → refresh page → session still valid)
- [ ] [TASK-048] [P1] [US5] Write integration test for session expiration at `backend/tests/integration/test_register_login_flow.py` (create expired session → access protected endpoint → 401 Unauthorized)

**Parallel Execution**:
```
|| TASK-039, TASK-040 (session validation + revocation, independent service methods)
→ || TASK-041, TASK-043 (logout + session retrieval endpoints, can parallelize)
→ TASK-042, TASK-044 (endpoint enhancements)
→ || TASK-045, TASK-046, TASK-047, TASK-048 (all tests, independent)
```

---

## Phase 4: User Story 6 - Protected Resource Access (US6) - P1

**Purpose**: Authenticated users can access protected resources; unauthenticated users cannot.

**Dependencies**: US1 (session creation), US2 (login), US5 (session validation)

**Acceptance Scenarios**:
1. Authenticated user + valid session → protected endpoint succeeds + returns data
2. Unauthenticated user (no session) → protected endpoint → 401 Unauthorized + clear error
3. Expired session → protected endpoint → 401 Unauthorized + "Session expired" message
4. Tampered session token → protected endpoint → 401 Unauthorized + security error

**Independent Test**: With active session → call protected endpoint → verify 200 OK; Without session → call same endpoint → verify 401 Unauthorized

### Tasks

- [ ] [TASK-049] [P1] [US6] Create `get_current_user` dependency at `backend/src/api/deps.py` implementing research.md:327-363 (extract cookie, validate session, return user)
- [ ] [TASK-050] [P1] [US6] Add 401 error handling for missing session in `get_current_user` dependency
- [ ] [TASK-051] [P1] [US6] Add 401 error handling for expired session in `get_current_user` dependency with "Session expired" message
- [ ] [TASK-052] [P1] [US6] Add 401 error handling for revoked session in `get_current_user` dependency with "Session invalid" message
- [ ] [TASK-053] [P1] [US6] Create example protected endpoint GET `/api/user/profile` at `backend/src/api/routes/user.py` using `get_current_user` dependency
- [ ] [TASK-054] [P1] [US6] Write integration test for protected endpoint with valid session at `backend/tests/integration/test_protected_endpoints.py` (login → access profile → 200 OK)
- [ ] [TASK-055] [P1] [US6] Write integration test for protected endpoint without session at `backend/tests/integration/test_protected_endpoints.py` (no cookie → access profile → 401 Unauthorized)
- [ ] [TASK-056] [P1] [US6] Write integration test for protected endpoint with expired session at `backend/tests/integration/test_protected_endpoints.py` (create expired session → access profile → 401 Unauthorized)
- [ ] [TASK-057] [P1] [US6] Write integration test for protected endpoint with tampered token at `backend/tests/integration/test_protected_endpoints.py` (modify cookie → access profile → 401 Unauthorized)

**Parallel Execution**:
```
TASK-049 (get_current_user dependency skeleton)
→ || TASK-050, TASK-051, TASK-052 (error handling branches, can parallelize)
→ TASK-053 (example protected endpoint, depends on TASK-049)
→ || TASK-054, TASK-055, TASK-056, TASK-057 (all tests, independent)
```

---

## Phase 5: User Story 3 - Google OAuth Signup (US3) - P2

**Purpose**: Users can quickly create accounts using Google OAuth.

**Dependencies**: US1 (User model, session creation), US5 (session creation)

**Acceptance Scenarios**:
1. Click "Sign up with Google" → authorize → new account created + logged in
2. Google email already exists (from email/password signup) → Google account linked to existing user + logged in
3. User cancels OAuth at Google → redirected to signup page + no account created
4. Invalid OAuth state token → authentication fails with security error

**Independent Test**: Click "Sign up with Google" → complete OAuth flow → verify new user account created in database → verify Google OAuth account linked → verify session established

### Tasks

- [ ] [TASK-058] [P2] [US3] Create OAuthAccount SQLAlchemy model at `backend/src/models/oauth_account.py` from data-model.md:359-433
- [ ] [TASK-059] [P2] [US3] Install and configure Authlib at `backend/src/config.py` with Google OAuth client from research.md:124-136
- [ ] [TASK-060] [P2] [US3] Implement OAuth state generation and validation at `backend/src/services/oauth.py` (cryptographic random, server-side storage)
- [ ] [TASK-061] [P2] [US3] Create OAuth initiation endpoint GET `/api/auth/oauth/google` at `backend/src/api/routes/oauth.py` implementing spec.md:643-655 (generate state, redirect to Google)
- [ ] [TASK-062] [P2] [US3] Create OAuth callback endpoint GET `/api/auth/oauth/google/callback` at `backend/src/api/routes/oauth.py` implementing spec.md:657-698
- [ ] [TASK-063] [P2] [US3] Implement OAuth token exchange in callback (authorization code → access/ID/refresh tokens) from research.md:124-136
- [ ] [TASK-064] [P2] [US3] Extract user profile from Google ID token (sub, email, name) in callback
- [ ] [TASK-065] [P2] [US3] Implement account linking logic at `backend/src/services/oauth.py` (check email match → link to existing user or create new user) from spec.md:FR-005, FR-020
- [ ] [TASK-066] [P2] [US3] Create OAuth account record in database after successful OAuth flow
- [ ] [TASK-067] [P2] [US3] Create session and set cookie in OAuth callback (reuse US1 session logic)
- [ ] [TASK-068] [P2] [US3] Write unit test for OAuth state generation/validation at `backend/tests/unit/test_oauth_service.py` (generate → validate → True, tampered → False)
- [ ] [TASK-069] [P2] [US3] Write integration test for OAuth signup flow at `backend/tests/integration/test_oauth_flow.py` (mock Google OAuth → new user created → session established)
- [ ] [TASK-070] [P2] [US3] Write integration test for OAuth account linking at `backend/tests/integration/test_oauth_flow.py` (create user via email/password → OAuth with same email → accounts linked)
- [ ] [TASK-071] [P2] [US3] Write integration test for OAuth state validation at `backend/tests/integration/test_oauth_flow.py` (invalid state → 400 Bad Request)

**Parallel Execution**:
```
TASK-058 (OAuthAccount model)
→ TASK-059 (Authlib configuration)
→ || TASK-060, TASK-061 (OAuth state generation + initiation endpoint, can parallelize with state service first)
→ TASK-062 (callback endpoint skeleton)
→ || TASK-063, TASK-064, TASK-065, TASK-066, TASK-067 (callback logic, sequential: token exchange → profile extraction → account linking → create OAuth record → session)
→ || TASK-068, TASK-069, TASK-070, TASK-071 (all tests, independent)
```

---

## Phase 6: User Story 4 - Google OAuth Login (US4) - P2

**Purpose**: Users who signed up with Google can log in using Google OAuth.

**Dependencies**: US3 (OAuth flow, account linking)

**Acceptance Scenarios**:
1. User previously signed up with Google → click "Login with Google" → authorize → logged into existing account
2. Google account not registered → complete OAuth login → new account created (same as signup)
3. OAuth error at Google → redirected back with error → clear error message + retry option

**Independent Test**: For user who signed up via OAuth → logout → click "Login with Google" → complete OAuth → verify logged into same account (same user ID) → verify session established

### Tasks

- [ ] [TASK-072] [P2] [US4] Add existing user lookup by OAuth provider in callback at `backend/src/api/routes/oauth.py` (check oauth_accounts.provider + provider_account_id)
- [ ] [TASK-073] [P2] [US4] Handle OAuth login vs signup distinction in callback (if oauth_account exists → login, else → signup) from research.md:124-136
- [ ] [TASK-074] [P2] [US4] Add error handling for OAuth provider errors in callback (user cancels, access_denied) from spec.md:668-690
- [ ] [TASK-075] [P2] [US4] Write integration test for OAuth login (existing user) at `backend/tests/integration/test_oauth_flow.py` (OAuth signup → logout → OAuth login → same user ID)
- [ ] [TASK-076] [P2] [US4] Write integration test for OAuth error handling at `backend/tests/integration/test_oauth_flow.py` (simulate access_denied → 400 Bad Request)

**Parallel Execution**:
```
|| TASK-072, TASK-073 (OAuth login logic, related but can be implemented together)
→ TASK-074 (error handling)
→ || TASK-075, TASK-076 (tests, independent)
```

---

## Phase 7: Frontend Integration (P2)

**Purpose**: Docusaurus frontend provides UI for all authentication flows.

**Dependencies**: All backend user stories (US1-US6)

**Acceptance**: Users can signup, login, logout, and access protected resources via frontend UI.

### Tasks

- [ ] [TASK-077] [P2] [FRONTEND] Create auth utility functions at `frontend/src/utils/auth.ts` implementing quickstart.md:358-403 (register, login, logout, getCurrentSession, loginWithGoogle)
- [ ] [TASK-078] [P2] [FRONTEND] Create `useAuth` React hook at `frontend/src/hooks/useAuth.ts` for auth state management (fetch session on mount, provide user/session state)
- [ ] [TASK-079] [P2] [FRONTEND] Create RegisterForm component at `frontend/src/components/auth/RegisterForm.tsx` (name, email, password inputs + validation + submit to `/api/auth/register`)
- [ ] [TASK-080] [P2] [FRONTEND] Create LoginForm component at `frontend/src/components/auth/LoginForm.tsx` (email, password inputs + submit to `/api/auth/login`)
- [ ] [TASK-081] [P2] [FRONTEND] Create GoogleLoginButton component at `frontend/src/components/auth/GoogleLoginButton.tsx` (redirect to `/api/auth/oauth/google`)
- [ ] [TASK-082] [P2] [FRONTEND] Create LogoutButton component at `frontend/src/components/auth/LogoutButton.tsx` (call `/api/auth/logout` + clear local state)
- [ ] [TASK-083] [P2] [FRONTEND] Create signup page at `frontend/pages/signup.tsx` using RegisterForm and GoogleLoginButton
- [ ] [TASK-084] [P2] [FRONTEND] Create login page at `frontend/pages/login.tsx` using LoginForm and GoogleLoginButton
- [ ] [TASK-085] [P2] [FRONTEND] Add protected route wrapper using `useAuth` hook (redirect to login if not authenticated)
- [ ] [TASK-086] [P2] [FRONTEND] Add error display for authentication errors (validation, rate limiting, OAuth errors)

**Parallel Execution**:
```
TASK-077 (auth utility functions)
→ TASK-078 (useAuth hook, depends on TASK-077)
→ || TASK-079, TASK-080, TASK-081, TASK-082 (all form components, independent)
→ || TASK-083, TASK-084 (pages, depends on components)
→ || TASK-085, TASK-086 (protected route wrapper + error handling, can parallelize)
```

---

## Phase 8: Security & Production Hardening (P3)

**Purpose**: Production-ready security, monitoring, and deployment.

**Dependencies**: All P1 and P2 tasks

**Acceptance**: Rate limiting active, audit logging working, security tests passing, CI/CD deployed.

### Tasks

- [ ] [TASK-087] [P3] [SECURITY] Install and configure slowapi rate limiter at `backend/src/main.py` from research.md:237-249
- [ ] [TASK-088] [P3] [SECURITY] Add Redis connection for rate limiting (Upstash or local) in config from research.md:429-456
- [ ] [TASK-089] [P3] [SECURITY] Apply rate limiting to `/api/auth/register` (5 requests per 10 minutes per IP)
- [ ] [TASK-090] [P3] [SECURITY] Apply rate limiting to `/api/auth/login` (5 requests per 10 minutes per email + IP tracking)
- [ ] [TASK-091] [P3] [SECURITY] Apply rate limiting to `/api/auth/oauth/*` (20 requests per minute per IP)
- [ ] [TASK-092] [P3] [SECURITY] Implement audit logging service at `backend/src/services/logging.py` (log signup, login, logout, failed attempts with timestamp, user, IP, result)
- [ ] [TASK-093] [P3] [SECURITY] Add audit log calls to all authentication endpoints (register, login, logout, OAuth callback)
- [ ] [TASK-094] [P3] [SECURITY] Create audit_logs table in Alembic migration (if storing logs in database instead of files)
- [ ] [TASK-095] [P3] [SECURITY] Write security tests at `backend/tests/security/test_auth_security.py` (SQL injection attempts, XSS attempts, CSRF validation, session fixation)
- [ ] [TASK-096] [P3] [SECURITY] Write performance test for 1000 concurrent requests at `backend/tests/performance/test_load.py` (verify SC-004: no degradation)
- [ ] [TASK-097] [P3] [DEPLOYMENT] Create `.env.production` template with production configuration (SECURE_COOKIES=true, HTTPS enforced)
- [ ] [TASK-098] [P3] [DEPLOYMENT] Create GitHub Actions CI/CD pipeline at `.github/workflows/test.yml` (run pytest, linting, type checking on PR)
- [ ] [TASK-099] [P3] [DEPLOYMENT] Create deployment configuration for Railway/Fly.io/Vercel (backend + frontend deployment)
- [ ] [TASK-100] [P3] [DEPLOYMENT] Add HSTS headers to production FastAPI app from research.md:404-420
- [ ] [TASK-101] [P3] [DOCS] Generate OpenAPI documentation at `/api/docs` (FastAPI built-in Swagger UI)
- [ ] [TASK-102] [P3] [DOCS] Create production deployment guide at `docs/deployment.md` (environment setup, database migrations, secrets management)
- [ ] [TASK-103] [P3] [DOCS] Verify all security checklist items from plan.md:475-491 (15 items)

**Parallel Execution**:
```
|| TASK-087, TASK-088 (rate limiting setup)
→ || TASK-089, TASK-090, TASK-091 (apply rate limits to endpoints, can parallelize)
|| TASK-092 (audit logging service)
→ TASK-093 (add audit log calls, depends on TASK-092)
→ TASK-094 (audit_logs table if needed)
|| TASK-095, TASK-096 (security + performance tests, independent)
|| TASK-097, TASK-098, TASK-099, TASK-100 (deployment tasks, can parallelize)
|| TASK-101, TASK-102, TASK-103 (documentation tasks, can parallelize)
```

---

## Phase 9: Contract Testing & Final Validation (P3)

**Purpose**: Ensure API contracts match specifications and all success criteria are met.

**Dependencies**: All implementation tasks (Phases 1-8)

**Acceptance**: All contract tests passing, all success criteria verified, ready for production.

### Tasks

- [ ] [TASK-104] [P3] [TESTING] Write contract tests at `backend/tests/contract/test_api_contracts.py` validating all responses match `contracts/auth-api.yaml`
- [ ] [TASK-105] [P3] [TESTING] Write contract tests validating protected endpoint pattern matches `contracts/protected-endpoints.yaml`
- [ ] [TASK-106] [P3] [TESTING] Verify SC-001: Email/password signup completes in <60s (measure with test)
- [ ] [TASK-107] [P3] [TESTING] Verify SC-002: OAuth signup completes in <30s (measure with test)
- [ ] [TASK-108] [P3] [TESTING] Verify SC-003: 95% of logins succeed within 2s (load test with valid credentials)
- [ ] [TASK-109] [P3] [TESTING] Verify SC-004: 1000 concurrent requests without degradation (load test)
- [ ] [TASK-110] [P3] [TESTING] Verify SC-005: Zero plain-text passwords in database (audit database dump)
- [ ] [TASK-111] [P3] [TESTING] Verify SC-006: Protected endpoints return 401 within 500ms (measure with test)
- [ ] [TASK-112] [P3] [TESTING] Verify SC-007: Sessions persist for 30 days (create session, verify expires_at)
- [ ] [TASK-113] [P3] [TESTING] Verify SC-008: Rate limiting blocks after 5 failed attempts (integration test)
- [ ] [TASK-114] [P3] [TESTING] Verify SC-009: OAuth state validation prevents CSRF (security test with invalid state)
- [ ] [TASK-115] [P3] [TESTING] Verify SC-011: All auth events logged (check audit logs for signup, login, logout)
- [ ] [TASK-116] [P3] [TESTING] Verify SC-012: Multi-device sessions with independent logout (create 2 sessions, logout one, verify other still valid)

**Parallel Execution**:
```
|| TASK-104, TASK-105 (contract tests, can run in parallel)
|| TASK-106, TASK-107, TASK-108, TASK-109, TASK-110, TASK-111, TASK-112, TASK-113, TASK-114, TASK-115, TASK-116 (all success criteria validations, independent tests)
```

---

## Dependency Graph (User Story Level)

```
SETUP (Phase 0)
  ↓
  ├─→ US1 (Email/Password Signup) [P1]
  │    ↓
  │    ├─→ US2 (Email/Password Login) [P1]
  │    │    ↓
  │    │    └─→ US5 (Session Management) [P1]
  │    │         ↓
  │    │         └─→ US6 (Protected Resources) [P1]
  │    │
  │    └─→ US3 (Google OAuth Signup) [P2]
  │         ↓
  │         └─→ US4 (Google OAuth Login) [P2]
  │
  └─→ (All US1-US6) → Frontend Integration [P2]
       ↓
       └─→ Security & Hardening [P3]
            ↓
            └─→ Contract Testing & Validation [P3]
```

**Critical Path** (longest dependency chain):
```
SETUP → US1 → US2 → US5 → US6 → Frontend → Security → Testing
```

**Parallel Execution Opportunities**:
- After SETUP completes: Start US1 immediately
- After US1 completes: Start US2 AND US3 in parallel
- After US2 completes: Start US5
- After US3 completes: Start US4
- After US6 completes: Start Frontend Integration
- After Frontend completes: Start Security & Hardening
- Security tasks (TASK-087 to TASK-103) can be parallelized in groups

---

## Testing Strategy Summary

**Unit Tests** (23 tasks):
- Password hashing (TASK-023, TASK-035)
- Session token generation (TASK-024)
- Session validation (TASK-045)
- OAuth state generation/validation (TASK-068)

**Integration Tests** (24 tasks):
- Registration flow (TASK-025, TASK-026, TASK-027)
- Login flow (TASK-036, TASK-037, TASK-038)
- Logout flow (TASK-046, TASK-047, TASK-048)
- Protected endpoints (TASK-054, TASK-055, TASK-056, TASK-057)
- OAuth flow (TASK-069, TASK-070, TASK-071, TASK-075, TASK-076)

**Security Tests** (1 task):
- OWASP Top 10 validation (TASK-095)

**Performance Tests** (1 task):
- Concurrent request handling (TASK-096)

**Contract Tests** (2 tasks):
- API contract validation (TASK-104, TASK-105)

**Success Criteria Validation** (13 tasks):
- SC-001 to SC-012 (TASK-106 to TASK-116)

**Total Tests**: 64 test tasks

---

## Success Criteria Mapping

| Success Criteria | Validated By | Task ID |
|------------------|--------------|---------|
| SC-001: Signup <60s | Performance test | TASK-106 |
| SC-002: OAuth signup <30s | Performance test | TASK-107 |
| SC-003: 95% logins in <2s | Load test | TASK-108 |
| SC-004: 1000 concurrent users | Load test | TASK-109 |
| SC-005: Zero plain-text passwords | Database audit | TASK-110 |
| SC-006: 401 responses <500ms | Performance test | TASK-111 |
| SC-007: Sessions persist 30 days | Integration test | TASK-112 |
| SC-008: Rate limiting blocks after 5 attempts | Integration test | TASK-113 |
| SC-009: OAuth state prevents CSRF | Security test | TASK-114 |
| SC-010: 90% first-attempt success | Manual QA | N/A (UX metric) |
| SC-011: Complete audit trail | Integration test | TASK-115 |
| SC-012: Multi-device sessions | Integration test | TASK-116 |

---

## Estimated Task Count

- **Phase 0 (SETUP)**: 10 tasks
- **Phase 1 (US1)**: 17 tasks
- **Phase 2 (US2)**: 11 tasks
- **Phase 3 (US5)**: 10 tasks
- **Phase 4 (US6)**: 9 tasks
- **Phase 5 (US3)**: 14 tasks
- **Phase 6 (US4)**: 5 tasks
- **Phase 7 (Frontend)**: 10 tasks
- **Phase 8 (Security)**: 17 tasks
- **Phase 9 (Testing)**: 13 tasks

**Total**: 116 tasks

---

## Implementation Priorities

### Sprint 1: P1 MVP (SETUP + US1 + US2 + US5 + US6)
**Tasks**: TASK-001 to TASK-057 (57 tasks)
**Acceptance**: Users can signup, login, logout with email/password. Protected endpoints work. Sessions managed correctly.

### Sprint 2: P2 Enhanced UX (US3 + US4 + Frontend)
**Tasks**: TASK-058 to TASK-086 (29 tasks)
**Acceptance**: Users can signup/login with Google. Account linking works. Frontend UI integrated.

### Sprint 3: P3 Production Ready (Security + Testing)
**Tasks**: TASK-087 to TASK-116 (30 tasks)
**Acceptance**: Rate limiting active. Audit logs working. All security tests passing. Contract tests passing. Ready for production deployment.

---

## Next Steps

1. **Review and approve tasks.md**: Ensure all tasks align with spec.md user stories and plan.md architecture
2. **Set up project**: Run TASK-001 to TASK-010 (Phase 0: Setup)
3. **Begin implementation**: Start with US1 (Email/Password Signup) as first user story
4. **Iterate by user story**: Complete and test each user story independently before moving to next
5. **Deploy to staging**: After Sprint 1 (P1 MVP), deploy to staging environment for testing
6. **Production deployment**: After Sprint 3 (P3), deploy to production with monitoring

---

**Tasks Generated**: 2025-12-14
**Source Documents**: [spec.md](./spec.md), [plan.md](./plan.md), [data-model.md](./data-model.md), [contracts/auth-api.yaml](./contracts/auth-api.yaml), [research.md](./research.md), [quickstart.md](./quickstart.md)
**Ready for**: Implementation via `/sp.implement` command
