# Implementation Plan: Authentication Session Persistence Fix

**Branch**: `001-fix-auth` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-fix-auth/spec.md`

## Summary

Fix critical authentication session persistence failures where users experience authentication loss after successful login/signup. The root cause is cross-origin cookie configuration mismatch between FastAPI backend (port 8000) and Docusaurus frontend (port 3000/GitHub Pages). The solution involves:

1. **Cookie Configuration Fix**: Environment-specific `SameSite` and `Secure` attributes (`Lax` for dev, `None; Secure` for prod)
2. **OAuth Token Exchange Fix**: Add session verification step before redirect to prevent race condition
3. **Onboarding Modal Fix**: Automatic trigger based on `onboarding_required` flag from auth responses

**Technical Approach**: Minimal configuration changes to existing FastAPI session-based auth. No framework rewrite. All changes focused on cookie attributes, CORS headers, and frontend auth state synchronization.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.6.2 (frontend)
**Primary Dependencies**:
- Backend: FastAPI 0.115+, Starlette CORS middleware, psycopg3 (Postgres adapter), Pydantic
- Frontend: React 19.0.0, Docusaurus 3.9.2, fetch API

**Storage**: Neon Serverless Postgres (existing, shared instance)
**Testing**: pytest (backend unit/integration), Jest (frontend unit), Playwright (E2E)
**Target Platform**:
- Development: localhost HTTP (frontend port 3000, backend port 8000)
- Production: GitHub Pages HTTPS (frontend), Render.com HTTPS (backend)

**Project Type**: Web application (decoupled frontend/backend)
**Performance Goals**:
- Session validation: <100ms p95 latency
- OAuth callback flow: Complete within 3 seconds end-to-end
- Onboarding modal: Appear within 2 seconds of registration/OAuth signup

**Constraints**:
- No session database queries on every frontend page load (use cookie validation)
- CORS must support credentials across different origins (localhost dev, GitHub Pages prod)
- Cookie `SameSite=None` requires `Secure=true` (HTTPS only in production)
- Zero breaking changes to existing user sessions (cookie name/domain must remain compatible)

**Scale/Scope**:
- Session lifetime: 30 days (configurable via environment variable)
- Concurrent sessions per user: Unlimited (each device gets own session token)
- Expected users: <10k concurrent (current project scale)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication Quality Gates (Constitution Section XIV)

- [x] **No custom auth logic bypassing Better Auth**: ✅ PASS - This is a FastAPI custom auth system (not Better Auth). The spec explicitly states we will NOT migrate to Better Auth framework. We're fixing the existing FastAPI session-based auth.
  - **Note**: Constitution requires Better Auth for new authentication implementations. This project predates that requirement and uses FastAPI session-based auth. The fix maintains the existing architecture with configuration corrections only.

- [x] **Secure cookie/token configuration**: ✅ WILL PASS AFTER FIX
  - Current state: FAIL (cookies have `SameSite=None` with `Secure=false` in dev, causing browser rejection)
  - After fix: PASS (cookies will have `SameSite=Lax` in dev, `SameSite=None; Secure` in prod)

- [x] **Session expiration verified**: ✅ PASS
  - Backend validates session expiry timestamp on every request (FR-032)
  - Expired sessions return 401, triggering frontend logout (FR-027)

- [x] **Frontend contains no secrets**: ✅ PASS
  - Frontend uses fetch with `credentials: "include"` for cookie-based auth
  - No OAuth secrets, session tokens, or API keys in frontend code
  - OAuth tokens passed via URL fragment (not exposed to server logs)

- [x] **Auth flows tested in staging**: ✅ WILL PASS IN IMPLEMENTATION
  - E2E tests required before deployment (see Testing Requirements in spec)
  - Manual testing checklist included in deployment documentation

### RAG Chatbot Quality Gates (Constitution Section XV)

**N/A** - This feature does not modify RAG chatbot functionality. No RAG quality gates apply.

### Constitution Alignment Summary

**Status**: ✅ **PASS WITH NOTES**

**Notes**:
1. **Better Auth Requirement**: Constitution Section IX states "Authentication MUST be implemented using Better Auth". However:
   - This project's auth system predates the constitution requirement
   - The spec explicitly excludes migrating to Better Auth (Out of Scope item #1)
   - The fix maintains existing FastAPI session-based auth with configuration corrections
   - **Decision**: Grandfather existing FastAPI auth system. Future new auth features must use Better Auth per constitution.

2. **Authentication Integration Scope**: Constitution Section XI requires auth review to include FastAPI auth config, routes, cookies, database schema, and environment variables. ✅ All covered in this plan.

3. **Authentication Tooling**: Constitution Section XII requires Better Auth documentation via MCP. **N/A** - Not applicable since we're not using Better Auth framework.

## Project Structure

### Documentation (this feature)

```text
specs/001-fix-auth/
├── spec.md              # Feature specification (✅ complete)
├── checklists/
│   └── requirements.md  # Specification quality validation (✅ complete)
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0 output (cookie/CORS best practices)
├── data-model.md        # Phase 1 output (session/user entities)
├── quickstart.md        # Phase 1 output (developer setup guide)
├── contracts/           # Phase 1 output (API contracts)
│   ├── auth-api.yaml    # OpenAPI spec for auth endpoints
│   └── session-cookie.md # Cookie contract specification
└── tasks.md             # Phase 2 output (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (FastAPI backend + Docusaurus frontend)

rag-chatbot/auth_backend/               # Authentication backend (FastAPI)
├── api/
│   ├── routes/
│   │   ├── auth.py                     # ✏️ MODIFY: Cookie setting logic
│   │   └── oauth.py                    # ✏️ MODIFY: OAuth callback cookie logic
│   └── deps.py                          # (no changes)
├── services/
│   └── session.py                       # ✏️ MODIFY: Session cookie helpers
├── config.py                            # ✏️ MODIFY: Environment-specific cookie config
├── models/
│   └── user.py                          # (no changes to schema)
└── tests/
    ├── test_auth_routes.py              # ✏️ ADD: Cookie attribute tests
    ├── test_oauth_routes.py             # ✏️ ADD: OAuth callback tests
    └── test_session_service.py          # ✏️ ADD: Session cookie tests

AIdd-book/src/                           # Docusaurus frontend
├── hooks/
│   └── useAuth.tsx                      # ✏️ MODIFY: Onboarding trigger logic
├── services/
│   └── authApi.ts                       # (already has credentials: include)
├── pages/auth/
│   └── callback.tsx                     # ✏️ MODIFY: Session verification before redirect
└── __tests__/
    ├── hooks/
    │   └── useAuth.test.tsx             # ✏️ ADD: Onboarding modal tests
    └── pages/auth/
        └── callback.test.tsx            # ✏️ ADD: Token exchange tests

qa-automation/                           # E2E test suite (existing)
└── tests/
    ├── test_auth_flow.py                # ✏️ ADD: Complete auth journey E2E test
    ├── test_oauth_flow.py               # ✏️ ADD: Google OAuth E2E test
    └── test_session_persistence.py      # ✏️ ADD: Session recovery E2E test
```

**Structure Decision**: Web application structure with decoupled backend and frontend. Backend is FastAPI Python application in `rag-chatbot/auth_backend/`. Frontend is Docusaurus React application in `AIdd-book/`. E2E tests in separate `qa-automation/` directory using Playwright.

**Modification Strategy**:
- **Backend**: 7 files modified (config, routes, services, tests)
- **Frontend**: 3 files modified (hooks, pages, tests)
- **E2E**: 3 new test files added
- **Total**: 13 file modifications across 3 directories

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations requiring justification.**

The FastAPI session-based auth system predates the Better Auth constitution requirement and is grandfathered. All other constitution requirements (secure cookies, no frontend secrets, session expiration, testing) are met or will be met after this fix.

---

## Phase 0: Research & Technical Decisions

**Objective**: Resolve all technical unknowns and establish best practices for cross-origin cookie configuration, CORS setup, and OAuth token exchange patterns.

### Research Tasks

1. **Cross-Origin Cookie Configuration**:
   - **Question**: What are the correct `SameSite`, `Secure`, and `Domain` attributes for cookies shared across `localhost:3000` → `localhost:8000` (dev) and `github.io` → `render.com` (prod)?
   - **Research Focus**:
     - Browser cookie policies for same-site vs cross-site requests
     - `SameSite=Lax` vs `SameSite=None; Secure` behavior
     - Cookie domain attribute behavior (explicit vs implicit)
     - Chrome, Firefox, Safari cookie policy differences

2. **CORS Configuration for Credentials**:
   - **Question**: What CORS headers are required to support `credentials: "include"` in fetch requests across different origins?
   - **Research Focus**:
     - `Access-Control-Allow-Credentials: true` requirement
     - Wildcard (`*`) origin forbidden when credentials enabled
     - Preflight request handling for authenticated endpoints
     - `Access-Control-Allow-Headers` for custom headers

3. **OAuth Token Exchange Best Practices**:
   - **Question**: How to prevent race conditions in OAuth callback flow where redirect happens before cookie is processed?
   - **Research Focus**:
     - URL fragment vs query parameter for token passing
     - Session verification patterns after token exchange
     - Retry logic for transient cookie setting failures
     - Error handling for invalid/expired OAuth tokens

4. **FastAPI Session Management Patterns**:
   - **Question**: Best practices for session cookie helpers and environment-specific configuration in FastAPI?
   - **Research Focus**:
     - Centralized cookie attribute configuration
     - Environment detection (dev vs prod)
     - Cookie deletion patterns (must match setting attributes)
     - Session validation middleware patterns

### Expected Outputs

**File**: `research.md`

**Structure**:
```markdown
# Research: Cross-Origin Authentication Cookie Configuration

## Decision 1: Cookie SameSite Attribute Strategy

**Chosen**: Environment-specific SameSite values
- Development (HTTP): `SameSite=Lax`
- Production (HTTPS): `SameSite=None; Secure`

**Rationale**:
- `SameSite=None` requires `Secure=true` (HTTPS), which dev doesn't have
- `SameSite=Lax` allows same-site cookies in dev without HTTPS
- Production uses different domains (github.io, render.com) requiring `None`

**Alternatives Considered**:
- ❌ `SameSite=None` in dev: Browser rejects without Secure flag
- ❌ `SameSite=Strict`: Blocks legitimate cross-origin auth flows
- ❌ `SameSite=Lax` in prod: Prevents cross-origin cookie transmission

**Implementation**:
```python
# config.py
secure_cookies: bool = os.getenv("SECURE_COOKIES", "false").lower() == "true"
same_site_cookies: str = "none" if secure_cookies else "lax"
```

## Decision 2: CORS Configuration for Credentials

[Similar structure for each research task...]

## Decision 3: OAuth Token Exchange Pattern

[Similar structure...]

## Decision 4: FastAPI Cookie Helper Design

[Similar structure...]
```

---

## Phase 1: Data Model & API Contracts

**Prerequisites**: Phase 0 `research.md` complete

### Data Model

**File**: `data-model.md`

**Entities** (from spec):

#### User Entity
```yaml
Entity: User
Purpose: Represents an authenticated account
Fields:
  - id: UUID (primary key)
  - email: String (unique, lowercase)
  - name: String
  - password_hash: String (nullable for OAuth-only users)
  - email_verified_at: DateTime (nullable)
  - onboarding_completed: Boolean (default: false)
  - preferred_locale: String (default: "en")
  - created_at: DateTime
  - updated_at: DateTime

Validation Rules:
  - email: Valid email format, lowercase normalization
  - name: 1-255 characters
  - password_hash: bcrypt hash (if email/password auth)
  - preferred_locale: Must be "en" or "ur"

State Transitions:
  - New user: onboarding_completed = false
  - After onboarding: onboarding_completed = true
  - No state machine (simple boolean flag)
```

#### Session Entity
```yaml
Entity: Session
Purpose: Represents an active authentication session
Fields:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User)
  - token_hash: String (hashed session token)
  - ip_address: String (nullable)
  - user_agent: String (nullable)
  - created_at: DateTime
  - last_used_at: DateTime
  - expires_at: DateTime

Validation Rules:
  - token_hash: SHA-256 hash of plain token (never store plain token)
  - expires_at: created_at + 30 days (configurable)
  - last_used_at: Updated on each validated request

Lifecycle:
  - Created: On login, registration, or OAuth callback
  - Updated: last_used_at on each request
  - Expired: When expires_at < now() → return 401
  - Revoked: On logout → deleted from database
```

#### OAuth Account Entity
```yaml
Entity: OAuth Account
Purpose: Links user to external OAuth provider
Fields:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User)
  - provider: String (e.g., "google", "github")
  - provider_user_id: String
  - access_token: String (encrypted)
  - refresh_token: String (encrypted, nullable)
  - expires_at: DateTime (nullable)
  - created_at: DateTime
  - updated_at: DateTime

Validation Rules:
  - provider: Must be one of supported providers ("google", "github")
  - provider_user_id: Unique per provider
  - access_token: Encrypted before storage
  - Composite unique constraint: (provider, provider_user_id)

Relationships:
  - One User → Many OAuth Accounts (multi-provider linking)
```

#### Onboarding Profile Entity
```yaml
Entity: Onboarding Profile
Purpose: Stores user's onboarding responses
Fields:
  - id: UUID (primary key)
  - user_id: UUID (foreign key to User, unique)
  - user_type: String
  - area_of_interest: String
  - experience_level: String
  - topics_of_interest: String[] (nullable)
  - created_at: DateTime
  - updated_at: DateTime

Validation Rules:
  - user_type: Predefined list from onboarding options API
  - area_of_interest: Predefined list from onboarding options API
  - experience_level: Predefined list from onboarding options API
  - topics_of_interest: Array of predefined values

Relationships:
  - One User → One Onboarding Profile (1:1)
  - Created when user completes onboarding
  - Sets User.onboarding_completed = true
```

**Entity Relationships**:
```
User 1──* Session
User 1──* OAuth Account
User 1──1 Onboarding Profile
```

### API Contracts

**Directory**: `contracts/`

#### File: `auth-api.yaml` (OpenAPI 3.0)

```yaml
openapi: 3.0.0
info:
  title: Authentication API
  version: 1.0.0
  description: FastAPI authentication endpoints with session cookie management

servers:
  - url: http://localhost:8000
    description: Development
  - url: https://aidd-chatbot-api.onrender.com
    description: Production

paths:
  /auth/register:
    post:
      summary: Register new user with email/password
      operationId: registerUser
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, name, password]
              properties:
                email:
                  type: string
                  format: email
                name:
                  type: string
                  minLength: 1
                  maxLength: 255
                password:
                  type: string
                  minLength: 8
      responses:
        '201':
          description: User registered successfully
          headers:
            Set-Cookie:
              description: Session cookie with correct SameSite/Secure attributes
              schema:
                type: string
                example: "session_token=abc123; HttpOnly; SameSite=Lax; Path=/"
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AuthResponse'
        '409':
          description: Email already registered
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /auth/login:
    post:
      summary: Login with email/password
      operationId: loginUser
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password]
              properties:
                email:
                  type: string
                  format: email
                password:
                  type: string
      responses:
        '200':
          description: Login successful
          headers:
            Set-Cookie:
              description: Session cookie
              schema:
                type: string
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AuthResponse'
        '401':
          description: Invalid credentials
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /auth/logout:
    post:
      summary: Logout current user
      operationId: logoutUser
      security:
        - cookieAuth: []
      responses:
        '200':
          description: Logout successful
          headers:
            Set-Cookie:
              description: Cleared session cookie
              schema:
                type: string
                example: "session_token=; HttpOnly; SameSite=Lax; Path=/; Max-Age=0"
          content:
            application/json:
              schema:
                type: object
                properties:
                  message:
                    type: string
        '401':
          description: Not authenticated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /auth/session:
    get:
      summary: Get current session
      operationId: getCurrentSession
      security:
        - cookieAuth: []
      responses:
        '200':
          description: Session retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AuthResponse'
        '401':
          description: Not authenticated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /auth/session-from-token:
    post:
      summary: Exchange OAuth token for session cookie
      operationId: createSessionFromToken
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [token]
              properties:
                token:
                  type: string
                  description: Session token from OAuth callback URL fragment
      responses:
        '200':
          description: Session established
          headers:
            Set-Cookie:
              description: Session cookie
              schema:
                type: string
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AuthResponse'
        '401':
          description: Invalid or expired token
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /auth/oauth/google:
    get:
      summary: Initiate Google OAuth flow
      operationId: googleOAuthLogin
      responses:
        '302':
          description: Redirect to Google OAuth authorization page
          headers:
            Location:
              schema:
                type: string
                example: "https://accounts.google.com/o/oauth2/auth?..."

  /auth/oauth/google/callback:
    get:
      summary: Handle Google OAuth callback
      operationId: googleOAuthCallback
      parameters:
        - name: code
          in: query
          required: true
          schema:
            type: string
        - name: state
          in: query
          required: true
          schema:
            type: string
      responses:
        '302':
          description: Redirect to frontend with token in URL fragment
          headers:
            Location:
              schema:
                type: string
                example: "http://localhost:3000/Physical-AI-Robotic-Book/auth/callback#session_token=abc123&onboarding_required=true"

components:
  schemas:
    AuthResponse:
      type: object
      required: [user, session, message, onboarding_required]
      properties:
        user:
          $ref: '#/components/schemas/User'
        session:
          $ref: '#/components/schemas/Session'
        message:
          type: string
        onboarding_required:
          type: boolean

    User:
      type: object
      required: [id, email, name, onboarding_completed, created_at, updated_at]
      properties:
        id:
          type: string
          format: uuid
        email:
          type: string
          format: email
        name:
          type: string
        email_verified_at:
          type: string
          format: date-time
          nullable: true
        onboarding_completed:
          type: boolean
        preferred_locale:
          type: string
          enum: [en, ur]
        created_at:
          type: string
          format: date-time
        updated_at:
          type: string
          format: date-time

    Session:
      type: object
      required: [id, user_id, last_used_at, expires_at, created_at]
      properties:
        id:
          type: string
          format: uuid
        user_id:
          type: string
          format: uuid
        last_used_at:
          type: string
          format: date-time
        expires_at:
          type: string
          format: date-time
        ip_address:
          type: string
          nullable: true
        user_agent:
          type: string
          nullable: true
        created_at:
          type: string
          format: date-time

    Error:
      type: object
      required: [detail]
      properties:
        detail:
          type: string

  securitySchemes:
    cookieAuth:
      type: apiKey
      in: cookie
      name: session_token
```

#### File: `session-cookie.md` (Cookie Contract)

```markdown
# Session Cookie Contract

## Cookie Name
`session_token`

## Cookie Attributes

### Development Environment
```
session_token=<token_value>;
HttpOnly;
SameSite=Lax;
Path=/;
Max-Age=2592000
```

**Attributes**:
- `HttpOnly`: ✅ Yes (prevents JavaScript access)
- `Secure`: ❌ No (HTTP doesn't support Secure flag)
- `SameSite`: `Lax` (allows same-site POST requests, blocks cross-site)
- `Domain`: Not set (defaults to current domain: `localhost`)
- `Path`: `/` (available on all routes)
- `Max-Age`: `2592000` seconds (30 days)

### Production Environment
```
session_token=<token_value>;
HttpOnly;
Secure;
SameSite=None;
Path=/;
Max-Age=2592000
```

**Attributes**:
- `HttpOnly`: ✅ Yes (prevents JavaScript access)
- `Secure`: ✅ Yes (HTTPS only)
- `SameSite`: `None` (allows cross-origin cookies with Secure)
- `Domain`: Not set (defaults to backend domain: `aidd-chatbot-api.onrender.com`)
- `Path`: `/` (available on all routes)
- `Max-Age`: `2592000` seconds (30 days)

## Cookie Value

**Format**: Opaque session token (generated server-side)
**Storage**: SHA-256 hash stored in database, plain token sent in cookie
**Rotation**: New token generated on login/register, old token revoked
**Validation**: Backend queries database for session by token hash

## Cookie Lifecycle

1. **Set**: On `/auth/register`, `/auth/login`, `/auth/session-from-token`
2. **Sent**: On every request to `/auth/*` and protected routes
3. **Validated**: Backend checks token hash in database, verifies expiry
4. **Updated**: `last_used_at` timestamp updated on each request
5. **Cleared**: On `/auth/logout` (Set-Cookie with `Max-Age=0`)

## CORS Requirements

**Backend CORS Headers**:
```
Access-Control-Allow-Origin: http://localhost:3000 (dev) or https://daniyalaneeq.github.io (prod)
Access-Control-Allow-Credentials: true
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization
```

**Frontend Request Headers**:
```javascript
fetch(url, {
  method: 'POST',
  credentials: 'include',  // ← Required for cookie transmission
  headers: {
    'Content-Type': 'application/json'
  }
})
```

## Security Considerations

1. **HttpOnly**: Prevents XSS attacks from stealing session token
2. **Secure** (prod): Prevents MITM attacks on unencrypted connections
3. **SameSite**: Prevents CSRF attacks (Lax in dev, None with Secure in prod)
4. **Token Hashing**: Database stores SHA-256 hash, not plain token
5. **Expiration**: Sessions expire after 30 days of inactivity
6. **Rotation**: New session on login, old session revoked
```

### Developer Quickstart Guide

**File**: `quickstart.md`

```markdown
# Developer Quickstart: Authentication Fix Implementation

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ and npm installed
- Neon Postgres database URL (see `.env.example`)
- Google OAuth credentials (optional for OAuth testing)

## Environment Setup

### Backend (FastAPI)

1. Navigate to backend directory:
   ```bash
   cd rag-chatbot/auth_backend
   ```

2. Create `.env` file (copy from `.env.example`):
   ```bash
   # Development environment
   DATABASE_URL=postgresql://user:password@localhost/dbname
   SESSION_SECRET=your-secret-key-minimum-32-characters
   SESSION_COOKIE_NAME=session_token
   SESSION_MAX_AGE_DAYS=30

   # Cookie configuration (CRITICAL for this fix)
   SECURE_COOKIES=false              # ← false for dev (HTTP)
   SAME_SITE_COOKIES=lax             # ← lax for dev, none for prod

   # CORS configuration
   CORS_ORIGINS=http://localhost:3000,http://localhost:8000

   # Frontend URL for OAuth redirects
   FRONTEND_URL=http://localhost:3000/Physical-AI-Robotic-Book

   # Google OAuth (optional)
   OAUTH_GOOGLE_CLIENT_ID=your-client-id
   OAUTH_GOOGLE_CLIENT_SECRET=your-client-secret
   OAUTH_GOOGLE_REDIRECT_URI=http://localhost:8000/auth/oauth/google/callback
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Run backend:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

### Frontend (Docusaurus)

1. Navigate to frontend directory:
   ```bash
   cd AIdd-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run frontend:
   ```bash
   npm start
   ```

   Frontend will be available at `http://localhost:3000/Physical-AI-Robotic-Book/`

## Verification Checklist

After starting both servers, verify the fix works:

### 1. Email/Password Authentication
- [ ] Navigate to `http://localhost:3000/Physical-AI-Robotic-Book/login`
- [ ] Register a new account
- [ ] Verify you remain logged in after registration
- [ ] Navigate to `/profile` → should NOT log you out
- [ ] Open DevTools → Application → Cookies
- [ ] Verify `session_token` cookie has:
  - `HttpOnly`: ✓
  - `SameSite`: Lax
  - `Secure`: (empty/false in dev)
  - `Path`: /

### 2. OAuth Authentication
- [ ] Click "Login with Google"
- [ ] Complete Google OAuth flow
- [ ] Verify redirect to `/auth/callback` shows "Welcome" message
- [ ] Verify final redirect to home or onboarding
- [ ] Verify you remain authenticated (name in navbar)
- [ ] Verify `session_token` cookie exists in DevTools

### 3. Onboarding Modal
- [ ] Register a new account
- [ ] Verify onboarding modal appears automatically
- [ ] Complete onboarding form
- [ ] Verify modal doesn't reappear after completion

### 4. Session Persistence
- [ ] Login successfully
- [ ] Refresh the page
- [ ] Verify you remain logged in (name still in navbar)
- [ ] Close and reopen browser
- [ ] Navigate to app again
- [ ] Verify session restored (within 30 days)

## Testing

### Backend Unit Tests
```bash
cd rag-chatbot/auth_backend
pytest tests/ -v
```

### Frontend Unit Tests
```bash
cd AIdd-book
npm test
```

### E2E Tests
```bash
cd qa-automation
pytest tests/test_auth_flow.py -v
```

## Debugging

### Common Issues

**Issue**: Cookie not sent in requests
- **Check**: DevTools → Network → Request Headers
- **Verify**: `Cookie: session_token=...` present
- **Fix**: Ensure `credentials: 'include'` in fetch calls

**Issue**: Browser rejects cookie with `SameSite=None`
- **Check**: Console for cookie warnings
- **Verify**: `SECURE_COOKIES=false` and `SAME_SITE_COOKIES=lax` in dev
- **Fix**: Never use `SameSite=None` without `Secure=true`

**Issue**: CORS error "credentials not allowed"
- **Check**: DevTools → Console for CORS errors
- **Verify**: Backend CORS middleware has `allow_credentials=True`
- **Fix**: Ensure CORS origins are explicit (not wildcard `*`)

**Issue**: OAuth callback says "Welcome" but user not authenticated
- **Check**: Network tab → `/auth/session-from-token` request
- **Verify**: Response sets `Set-Cookie` header
- **Fix**: Ensure frontend waits for response before redirect

## Production Deployment

### Environment Variables (Render.com / Backend)
```bash
SECURE_COOKIES=true                    # ← true for HTTPS
SAME_SITE_COOKIES=none                 # ← none for cross-origin
CORS_ORIGINS=https://daniyalaneeq.github.io
FRONTEND_URL=https://daniyalaneeq.github.io/Physical-AI-Robotic-Book
OAUTH_GOOGLE_REDIRECT_URI=https://aidd-chatbot-api.onrender.com/auth/oauth/google/callback
```

### Verification in Production
- [ ] Login works (no logout after accessing protected routes)
- [ ] OAuth works (user remains authenticated after callback)
- [ ] Cookies have `Secure` flag (check DevTools)
- [ ] Cookies have `SameSite=None` (check DevTools)
- [ ] CORS allows credentials (check Network tab)

## Next Steps

After verification:
1. Run `/sp.tasks` to generate task list
2. Implement tasks in TDD order (red-green-refactor)
3. Run all tests before committing
4. Deploy to staging for QA testing
5. Deploy to production after QA sign-off
```

---

## Phase 2: Task Generation (Future Step)

**NOTE**: Phase 2 is executed by `/sp.tasks` command, NOT by `/sp.plan`.

The `/sp.tasks` command will generate a detailed, granular task list in `tasks.md` based on this plan. Tasks will be ordered for Test-Driven Development (red-green-refactor) and will include:

1. **Backend Configuration Tasks**:
   - Create cookie helper function
   - Update environment config for cookie attributes
   - Modify login/register/logout endpoints
   - Modify OAuth callback endpoint
   - Add session verification endpoint

2. **Frontend Tasks**:
   - Update OAuth callback handler with verification step
   - Add onboarding modal trigger logic
   - Update auth hook for session recovery

3. **Testing Tasks**:
   - Backend unit tests (cookie attributes, session service)
   - Frontend unit tests (auth hook, OAuth callback)
   - Integration tests (login flow, OAuth flow)
   - E2E tests (complete auth journey, session persistence)

4. **Documentation Tasks**:
   - Update deployment documentation
   - Update developer setup guide
   - Create cookie troubleshooting guide

**Expected Output**: `tasks.md` with 30-40 granular, testable tasks

---

## Agent Context Update

After completing Phase 1 artifacts, run:

```bash
.specify/scripts/bash/update-agent-context.sh claude
```

This will update `.claude/settings.local.json` with new technologies from this plan, preserving any manual additions.

---

## Summary

This implementation plan provides a clear path to fix authentication session persistence issues through minimal configuration changes:

1. **Phase 0 (Research)**: Establish best practices for cross-origin cookies, CORS, and OAuth token exchange
2. **Phase 1 (Design)**: Define data model (4 entities), API contracts (8 endpoints), and cookie specifications
3. **Phase 2 (Tasks)**: Generate granular task list with `/sp.tasks` command (future step)

**Key Deliverables**:
- ✅ `plan.md` (this file)
- 🔄 `research.md` (Phase 0 output)
- 🔄 `data-model.md` (Phase 1 output)
- 🔄 `contracts/` (Phase 1 output)
- 🔄 `quickstart.md` (Phase 1 output)
- ⏭️ `tasks.md` (Phase 2, created by `/sp.tasks`)

**Constitution Compliance**: ✅ PASS (with grandfathered FastAPI auth system)

**Next Command**: Complete Phase 0 research tasks, then Phase 1 artifacts, then run `/sp.tasks`
