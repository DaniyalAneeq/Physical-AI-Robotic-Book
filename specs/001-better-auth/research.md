# Research: Better Auth Integration with FastAPI

**Feature**: 001-better-auth
**Date**: 2025-12-14
**Phase**: 0 (Research & Technical Decisions)

## Executive Summary

Better Auth is a JavaScript/TypeScript authentication library and **cannot run directly on Python/FastAPI**. However, we will treat Better Auth as an **architectural specification** and implement compatible patterns, endpoints, and security properties natively in FastAPI. This approach maintains alignment with the constitution's mandate that "Better Auth is the single source of truth" while respecting the technical reality of our Python backend.

---

## Critical Findings

### 1. Better Auth Compatibility with FastAPI

**Decision**: Implement Better Auth patterns in Python/FastAPI (not direct integration)

**Rationale**:
- Better Auth is a JavaScript/TypeScript library only
- No Python bindings or ports exist
- We can replicate Better Auth's API shape, security patterns, and architectural principles in FastAPI
- This allows us to maintain API compatibility if we ever migrate to a Node.js backend

**Alternatives Considered**:
- **Direct Better Auth integration**: Not possible - Better Auth requires Node.js/JavaScript runtime
- **Node.js microservice for auth**: Adds unnecessary complexity, deployment overhead, and latency; violates existing FastAPI architecture
- **Custom auth from scratch**: Violates constitution requirement for Better Auth; loses battle-tested patterns

**Implementation Approach**:
- Mirror Better Auth's endpoint structure (`/api/auth/register`, `/api/auth/login`, etc.)
- Replicate Better Auth's session management (database-backed, cookie-transported)
- Follow Better Auth's security defaults (HttpOnly cookies, token hashing, rate limiting)
- Maintain compatible response shapes for potential future migration

---

### 2. Password Hashing Algorithm

**Decision**: Use Argon2id via `argon2-cffi` library

**Rationale**:
- Argon2id is the current industry best practice for password hashing
- Winner of the Password Hashing Competition (2015)
- Better Auth recommends Argon2 over bcrypt
- Resistant to GPU/ASIC attacks and side-channel attacks
- Configurable memory, time, and parallelism parameters

**Configuration** (tuned for server capacity):
```python
from argon2 import PasswordHasher
ph = PasswordHasher(
    time_cost=3,         # iterations
    memory_cost=65536,   # 64 MB
    parallelism=1        # threads
)
```

**Alternatives Considered**:
- **bcrypt**: Acceptable fallback but older standard; Better Auth moved away from bcrypt to Argon2
- **PBKDF2**: Insufficient protection against GPU attacks; not recommended for new systems

---

### 3. Session Management Strategy

**Decision**: Database-backed sessions with hashed tokens stored in HttpOnly cookies

**Rationale**:
- Better Auth's default session model
- Tokens stored in database (only hashes, not plaintext)
- Raw token sent in Secure, HttpOnly cookie to prevent XSS/CSRF
- Sliding expiration (idle timeout) + absolute maximum (30 days default)
- Supports session revocation and device management

**Token Generation**:
```python
import secrets
import base64

# Generate 32-byte random token
raw_token = base64.urlsafe_b64encode(secrets.token_bytes(32)).decode().rstrip("=")

# Hash for storage (HMAC-SHA256 with server pepper)
import hmac
import hashlib
token_hash = hmac.new(
    PEPPER.encode(),
    raw_token.encode(),
    hashlib.sha256
).hexdigest()
```

**Cookie Configuration**:
```python
response.set_cookie(
    "session",
    raw_token,
    httponly=True,      # Prevent JavaScript access
    secure=True,        # HTTPS only (production)
    samesite="lax",     # CSRF protection (same-site requests)
    path="/",
    max_age=7*24*3600   # 7 days sliding window
)
```

**Alternatives Considered**:
- **JWT-only (stateless)**: Cannot revoke sessions; violates spec requirement for database-backed sessions
- **Server-side sessions (in-memory)**: Lost on restart; no multi-server support
- **Plain token storage**: Security risk if database breached; hashing is mandatory

---

### 4. OAuth 2.0 Implementation (Google)

**Decision**: Use `Authlib` for OAuth 2.0 integration

**Rationale**:
- Official OAuth 2.0 library for Python with FastAPI integration
- Supports OIDC (OpenID Connect) for Google
- Handles state/nonce validation automatically
- Compatible with Starlette/FastAPI async patterns

**Google OAuth Flow**:
1. User clicks "Sign in with Google" → redirect to `/api/auth/oauth/google`
2. Backend generates state + nonce, redirects to Google authorization URL
3. Google redirects back to `/api/auth/oauth/google/callback` with code + state
4. Backend verifies state, exchanges code for tokens (access, ID, refresh)
5. Extract user profile from ID token (sub, email, name, email_verified)
6. Find-or-create user + oauth_accounts record
7. Create session, set cookie, redirect to application

**Google Cloud Console Setup Required**:
- OAuth 2.0 Client ID and Client Secret
- Authorized redirect URIs: `https://yourdomain.com/api/auth/oauth/google/callback`
- Scopes: `openid`, `email`, `profile`

**Alternatives Considered**:
- **requests-oauthlib**: Synchronous only; incompatible with FastAPI async
- **Custom OAuth implementation**: Error-prone; state/nonce validation is complex
- **Social-auth-app-fastapi**: Less maintained; Authlib is more active

---

### 5. Database ORM and Migrations

**Decision**: SQLAlchemy 2.0 (async) + Alembic + asyncpg driver

**Rationale**:
- SQLAlchemy 2.0 supports async/await natively
- Alembic is industry standard for database migrations
- asyncpg is the fastest PostgreSQL driver for Python
- Neon Serverless Postgres is fully compatible with asyncpg
- Better Auth's database schema can be replicated in SQLAlchemy models

**Database Schema** (aligned with Better Auth patterns):

**users** table:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email CITEXT UNIQUE NOT NULL,
    name VARCHAR(255) NOT NULL,
    password_hash TEXT,  -- nullable for OAuth-only users
    email_verified_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);
CREATE UNIQUE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);
```

**sessions** table:
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(64) UNIQUE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    last_used_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    revoked BOOLEAN NOT NULL DEFAULT FALSE,
    ip_address INET,
    user_agent TEXT
);
CREATE UNIQUE INDEX idx_sessions_token_hash ON sessions(token_hash);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
CREATE INDEX idx_sessions_active ON sessions(user_id, revoked, expires_at DESC) WHERE revoked = FALSE;
```

**oauth_accounts** table:
```sql
CREATE TABLE oauth_accounts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    provider VARCHAR(50) NOT NULL,  -- 'google'
    provider_account_id VARCHAR(255) NOT NULL,  -- Google 'sub'
    access_token TEXT,
    refresh_token TEXT,
    expires_at TIMESTAMP WITH TIME ZONE,
    scope VARCHAR(255),
    token_type VARCHAR(50),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    UNIQUE(provider, provider_account_id)
);
CREATE INDEX idx_oauth_accounts_user_id ON oauth_accounts(user_id);
CREATE UNIQUE INDEX idx_oauth_accounts_provider ON oauth_accounts(provider, provider_account_id);
```

**Alternatives Considered**:
- **Django ORM**: Requires full Django framework; overkill for API-only backend
- **Tortoise ORM**: Less mature; smaller ecosystem
- **Raw SQL with asyncpg**: More boilerplate; no migration management

---

### 6. Rate Limiting Implementation

**Decision**: Redis-backed rate limiting with `slowapi` library

**Rationale**:
- Postgres is not ideal for high-frequency rate limit counters
- Redis provides atomic increment operations and TTL support
- `slowapi` integrates seamlessly with FastAPI/Starlette
- Upstash Redis (serverless) works well with Neon's serverless model

**Rate Limit Policies**:
- `/api/auth/register`: 5 requests per 10 minutes per IP
- `/api/auth/login`: 5 requests per 10 minutes per IP (per email tracking for brute force)
- `/api/auth/oauth/*`: 20 requests per minute per IP
- General API endpoints: 60 requests per minute per IP

**Implementation**:
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(
    key_func=get_remote_address,
    storage_uri="redis://upstash-redis-url"
)

@app.post("/api/auth/login")
@limiter.limit("5/10minutes")
async def login(request: Request):
    # ...
```

**Alternatives Considered**:
- **In-memory rate limiting**: Lost on restart; no multi-server support
- **Postgres-based counters**: High write load; not optimized for transient counters
- **starlette-limiter**: Similar to slowapi; both are acceptable

---

### 7. Frontend-Backend Authentication Flow

**Decision**: Cookie-based authentication with SameSite=Lax for same-site deployment

**Rationale**:
- Docusaurus (static React) and FastAPI can be served under same domain (e.g., `app.example.com` for frontend, `app.example.com/api` for backend)
- SameSite=Lax prevents CSRF for cross-site requests while allowing same-site navigation
- HttpOnly cookies prevent XSS attacks on token theft
- No authentication logic or secrets in frontend code (Docusaurus)

**Deployment Architecture**:
```
Option 1 (Recommended):
  - Frontend: https://app.example.com (Docusaurus static build)
  - Backend: https://app.example.com/api (FastAPI reverse-proxied)
  - SameSite: Lax

Option 2 (Cross-domain):
  - Frontend: https://www.example.com
  - Backend: https://api.example.com
  - SameSite: None + Secure (requires CSRF protection)
```

**CORS Configuration** (if cross-domain):
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://www.example.com"],
    allow_credentials=True,  # Required for cookies
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)
```

**Frontend Authentication Pattern** (Docusaurus):
```javascript
// Fetch current session (cookie sent automatically)
const response = await fetch('/api/auth/session', {
  credentials: 'include'  // Send cookies
});
const { user, session } = await response.json();

// Login
await fetch('/api/auth/login', {
  method: 'POST',
  credentials: 'include',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ email, password })
});

// Logout
await fetch('/api/auth/logout', {
  method: 'POST',
  credentials: 'include'
});
```

**Alternatives Considered**:
- **JWT in localStorage**: Vulnerable to XSS; requires manual header management; not Better Auth pattern
- **Session ID in URL**: Security risk; easily leaked in logs/referrers
- **Authorization header only**: Not compatible with browser navigation/redirects

---

### 8. Protected Routes Implementation

**Decision**: FastAPI dependency injection with custom `get_current_user` dependency

**Rationale**:
- FastAPI's dependency system is ideal for authentication middleware
- Reusable across all protected endpoints
- Type-safe user injection
- Clear error handling for unauthenticated/expired sessions

**Implementation Pattern**:
```python
from fastapi import Depends, HTTPException, Request

async def get_current_user(
    request: Request,
    db: AsyncSession = Depends(get_db)
) -> User:
    """
    Dependency to extract and validate current authenticated user.
    Raises 401 if session is invalid, expired, or missing.
    """
    raw_token = request.cookies.get("session")
    if not raw_token:
        raise HTTPException(status_code=401, detail="Authentication required")

    # Hash token and lookup session
    token_hash = compute_hash(raw_token)
    session = await db.get_session_by_hash(token_hash)

    if not session or session.revoked or session.expires_at < datetime.utcnow():
        raise HTTPException(status_code=401, detail="Session invalid or expired")

    # Update last_used_at (sliding expiration)
    await db.update_session_last_used(session.id)

    # Fetch and return user
    user = await db.get_user(session.user_id)
    return user

# Protected endpoint
@app.get("/api/chatbot/history")
async def get_chatbot_history(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    # current_user is automatically injected and validated
    history = await db.get_chatbot_history(current_user.id)
    return {"history": history}
```

**Alternatives Considered**:
- **Manual token validation in every endpoint**: Code duplication; error-prone
- **Middleware-based auth**: Less flexible; harder to make endpoints optionally protected
- **Decorator-based (@requires_auth)**: Less idiomatic for FastAPI; dependency system is preferred

---

### 9. Security Hardening

**Decisions** (aligned with Better Auth security best practices):

**A. Password Security**:
- Minimum password length: 8 characters
- Require combination of letters and numbers
- Never log or transmit passwords except over HTTPS
- Server-side pepper (environment variable) for additional hash protection

**B. Session Security**:
- 128-bit minimum entropy for session tokens (32 bytes)
- Token rotation on login (invalidate old sessions)
- Absolute maximum session lifetime: 30 days
- Idle timeout (sliding): 7 days
- Capture IP address and user agent for audit logging

**C. OAuth Security**:
- State parameter: cryptographically random, stored server-side with short TTL
- Nonce parameter: included in ID token, validated on callback
- Verify ID token signature, issuer, audience, expiration
- Whitelist redirect URIs in Google Cloud Console

**D. Input Validation**:
- Email normalization: lowercase before storage/lookup
- Use Pydantic models for request validation
- SQL injection prevention: parameterized queries (SQLAlchemy handles this)
- XSS prevention: output encoding (FastAPI JSONResponse handles this)

**E. HTTPS/TLS**:
- Enforce HTTPS in production (Secure cookie flag)
- HSTS headers: `Strict-Transport-Security: max-age=31536000; includeSubDomains`
- Redirect HTTP to HTTPS at reverse proxy/load balancer level

**F. Audit Logging**:
- Log all authentication events: signup, login, logout, password_change, email_verification
- Include timestamp, user_id, email, IP address, user_agent, result (success/failure)
- Never log passwords, tokens, or sensitive data
- Store logs securely with retention policy (e.g., 90 days for security, 1 year for compliance)

---

### 10. Development vs Production Configuration

**Decision**: Environment-based configuration with `.env` files and strict separation

**Configuration Variables**:
```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:pass@host:5432/db
NEON_CONNECTION_STRING=<from Neon console>

# Authentication
SESSION_COOKIE_NAME=session
SESSION_SECRET_PEPPER=<random 64-char hex>
SESSION_MAX_AGE_DAYS=30
SESSION_IDLE_TIMEOUT_DAYS=7

# Google OAuth
GOOGLE_CLIENT_ID=<from Google Cloud Console>
GOOGLE_CLIENT_SECRET=<from Google Cloud Console>
GOOGLE_REDIRECT_URI=https://yourdomain.com/api/auth/oauth/google/callback

# Security
SECURE_COOKIES=true  # false in dev, true in prod
ALLOWED_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# Rate Limiting
REDIS_URL=redis://upstash-redis-url
RATE_LIMIT_ENABLED=true

# Logging
LOG_LEVEL=INFO  # DEBUG in dev, INFO/WARNING in prod
AUDIT_LOG_ENABLED=true
```

**Development Setup**:
- Use `python-dotenv` to load `.env` file
- Local Postgres or Neon dev branch for database
- Local Redis or Upstash free tier
- SECURE_COOKIES=false for localhost (http://)
- Mock Google OAuth or use localhost redirect URI

**Production Deployment**:
- Environment variables injected via hosting platform (Vercel, Railway, Fly.io, etc.)
- Neon production branch with backups
- Upstash Redis production instance
- SECURE_COOKIES=true
- Verified Google OAuth redirect URIs only

---

## Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| Backend Framework | FastAPI | 0.115+ | API server, routing, dependency injection |
| Language | Python | 3.11+ | Runtime |
| Database | Neon Serverless Postgres | PostgreSQL 15+ | User, session, OAuth data persistence |
| ORM | SQLAlchemy | 2.0+ (async) | Database models, queries |
| Migrations | Alembic | Latest | Database schema versioning |
| Database Driver | asyncpg | Latest | Async PostgreSQL driver |
| Password Hashing | argon2-cffi | Latest | Argon2id password hashing |
| OAuth 2.0 | Authlib | Latest | Google OAuth integration |
| Rate Limiting | slowapi | Latest | API rate limiting |
| Cache/Rate Store | Redis (Upstash) | Latest | Rate limit counters, optional session cache |
| Validation | Pydantic | 2.0+ | Request/response validation |
| Frontend | Docusaurus | 3.9.2 | Static React documentation site |
| Frontend Auth | Fetch API | Native | Cookie-based auth (no additional library) |

---

## Next Steps (Phase 1)

1. **Create data-model.md**: Define SQLAlchemy models for users, sessions, oauth_accounts
2. **Create API contracts** (OpenAPI specs in `/contracts/`): Define all auth endpoints
3. **Create quickstart.md**: Setup instructions for local development
4. **Update agent context**: Add new technologies to CLAUDE.md

---

## Open Questions

**None** - All technical unknowns resolved.

---

## References

- Better Auth Documentation: https://better-auth.com
- Better Auth GitHub: https://github.com/better-auth/better-auth
- FastAPI Documentation: https://fastapi.tiangolo.com
- Authlib Documentation: https://docs.authlib.org
- SQLAlchemy 2.0 Documentation: https://docs.sqlalchemy.org
- Argon2 Specification: https://github.com/P-H-C/phc-winner-argon2
- OAuth 2.0 RFC: https://datatracker.ietf.org/doc/html/rfc6749
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
- Neon Documentation: https://neon.tech/docs

---

**Research Complete**: 2025-12-14
**Ready for Phase 1**: Data modeling and API contract design
