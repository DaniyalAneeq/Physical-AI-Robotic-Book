# Research: Authentication Refactor & Integration

**Feature**: 001-auth-refactor
**Date**: 2025-12-16
**Phase**: Phase 0 - Research & Discovery

## Executive Summary

This research phase investigates the technical approach for refactoring the authentication system to unify RAG chatbot and authentication backends into a single FastAPI application. Research focused on FastAPI routing patterns, authentication middleware implementation, database schema requirements, OAuth integration best practices, and onboarding flow patterns.

**Critical Finding**: The existing "Better Auth" implementation is actually a **custom Python authentication system** that follows Better Auth's conceptual patterns and database schema conventions. Better Auth is a TypeScript/JavaScript library with no official Python/FastAPI SDK. The existing implementation correctly follows industry best practices and Better Auth patterns.

## Research Areas

### 1. FastAPI Multi-Module Integration

**Decision**: Use FastAPI's `include_router()` with route prefixes to unify authentication and RAG backends.

**Rationale**:
- FastAPI's router system allows modular organization while running on a single ASGI application instance
- Route prefixes (`/api/*` for RAG, `/auth/*` for authentication) provide clear API namespacing
- Both modules can be developed independently and integrated at the application level
- Single FastAPI app means single port, single process, unified middleware chain

**Implementation Pattern**:
```python
# rag-chatbot/backend/app/main.py (UNIFIED ENTRY POINT)
from fastapi import FastAPI
from app.api import chatkit, conversations, health
from auth_backend.api.routes import auth, oauth, onboarding

app = FastAPI(title="Unified Backend")

# RAG endpoints with /api prefix
app.include_router(health.router, tags=["System"])
app.include_router(chatkit.router, prefix="/api", tags=["ChatKit"])
app.include_router(conversations.router, prefix="/api", tags=["Conversations"])

# Auth endpoints with /auth prefix
app.include_router(auth.router, prefix="/auth", tags=["Authentication"])
app.include_router(oauth.router, prefix="/auth/oauth", tags=["OAuth"])
app.include_router(onboarding.router, prefix="/auth/onboarding", tags=["Onboarding"])
```

**Alternatives Considered**:
- **Reverse proxy (nginx)**: Rejected because adds deployment complexity and doesn't solve development environment issues
- **API Gateway pattern**: Overkill for this scale, adds latency, requires additional infrastructure
- **Monolithic refactor**: Would require rewriting existing RAG backend, violates constraint to preserve RAG functionality

### 2. Authentication Middleware for Route Protection

**Decision**: Implement FastAPI dependency injection for route-level authentication with optional global middleware for blanket protection.

**Rationale**:
- FastAPI's dependency system provides fine-grained control over which routes require authentication
- Can specify different authentication levels (optional, required, admin-only) per route
- Middleware approach can be added later for global enforcement without changing route definitions
- Dependency injection provides better testability (can override dependencies in tests)

**Implementation Pattern**:
```python
# auth_backend/api/deps.py
from fastapi import Depends, HTTPException, Request
from auth_backend.services.session import get_session_from_cookie

async def get_current_user(request: Request):
    """Dependency: Require authenticated user."""
    session = await get_session_from_cookie(request)
    if not session or not session.user:
        raise HTTPException(status_code=401, detail="Authentication required")
    return session.user

async def get_current_user_with_onboarding(request: Request):
    """Dependency: Require authenticated user who completed onboarding."""
    user = await get_current_user(request)
    if not user.onboarding_completed:
        raise HTTPException(status_code=403, detail="Onboarding required")
    return user

# Usage in RAG endpoints
from auth_backend.api.deps import get_current_user_with_onboarding

@router.post("/chat")
async def chat(
    message: str,
    user: User = Depends(get_current_user_with_onboarding)  # Auth required + onboarding check
):
    # User is authenticated and onboarded
    pass
```

**Alternatives Considered**:
- **Global middleware only**: Less flexible, harder to exclude public routes, requires path matching logic
- **Decorator pattern**: Not idiomatic in FastAPI, less discoverable, harder to test
- **Manual session checks in routes**: Violates DRY, error-prone, inconsistent

### 3. OAuth Configuration and Redirect Handling

**Decision**: Configure OAuth providers with exact callback URLs matching the single-port architecture, use environment variables for URL configuration to support dev/prod environments.

**Rationale**:
- OAuth redirect URLs must match exactly (including port, protocol, path)
- Single port (8000) means all OAuth callbacks go to `http://localhost:8000/auth/oauth/{provider}/callback`
- Environment-based configuration allows different URLs for dev (localhost:8000) and prod (https://domain.com)
- State parameter provides CSRF protection during OAuth flow

**Configuration Requirements**:
```python
# .env configuration
OAUTH_GOOGLE_CLIENT_ID=...
OAUTH_GOOGLE_CLIENT_SECRET=...
OAUTH_GOOGLE_REDIRECT_URI=http://localhost:8000/auth/oauth/google/callback  # Dev
# Production: https://yourdomain.com/auth/oauth/google/callback

OAUTH_GITHUB_CLIENT_ID=...
OAUTH_GITHUB_CLIENT_SECRET=...
OAUTH_GITHUB_REDIRECT_URI=http://localhost:8000/auth/oauth/github/callback  # Dev

FRONTEND_URL=http://localhost:3000  # Docusaurus dev server
```

**OAuth Provider Console Setup**:
- **Google Cloud Console**: Add authorized redirect URI: `http://localhost:8000/auth/oauth/google/callback`
- **GitHub OAuth Apps**: Add callback URL: `http://localhost:8000/auth/oauth/github/callback`
- **Production**: Update both consoles with production URLs before deployment

**Alternatives Considered**:
- **Dynamic redirect URL detection**: Fragile, can be exploited for redirect attacks, not recommended by OAuth spec
- **Multiple callback URLs per environment**: Requires managing multiple OAuth app registrations, not necessary with env vars

### 4. Database Schema for Authentication and Onboarding

**Decision**: Extend existing authentication schema with `onboarding_completed` flag on `users` table and new `onboarding_profiles` table for profile data.

**Rationale**:
- Adding boolean flag to `users` table allows fast onboarding status checks without JOIN
- Separate `onboarding_profiles` table keeps user profile data normalized and allows future expansion
- Foreign key relationship ensures data integrity (cascade delete if user is removed)
- JSON/JSONB field for `topics_of_interest` provides flexibility for multi-select values

**Schema Design**:
```sql
-- Modify existing users table
ALTER TABLE users ADD COLUMN onboarding_completed BOOLEAN DEFAULT FALSE;

-- New onboarding_profiles table
CREATE TABLE onboarding_profiles (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    user_type VARCHAR(50) NOT NULL,  -- Student, Researcher, Teacher, Engineer, Other
    area_of_interest VARCHAR(100) NOT NULL,  -- Predefined options
    experience_level VARCHAR(20) NOT NULL,  -- Beginner, Intermediate, Advanced
    topics_of_interest JSONB,  -- ["ROS2", "NVIDIA Isaac", "Voice Control", ...]
    completed_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_onboarding_user_id ON onboarding_profiles(user_id);
CREATE INDEX idx_onboarding_completed ON onboarding_profiles(completed_at);
```

**Migration Strategy**:
- Use Alembic for schema migrations (existing pattern in `auth_backend/migrations/`)
- Migration file: `002_onboarding.py`
- For existing users, set `onboarding_completed = FALSE` (they must complete onboarding on next login)
- Provide data migration script if needed to backfill onboarding profiles for test users

**Alternatives Considered**:
- **Single table (denormalize)**: Rejected because adds many nullable columns to users table, harder to extend
- **EAV pattern**: Overkill for structured onboarding data, poor query performance
- **Separate database**: Violates constraint to use single Neon Postgres instance

### 5. Session Management and Cookie Configuration

**Decision**: Use HTTP-only secure cookies for session tokens, implement sliding session expiration, provide token refresh endpoint.

**Rationale**:
- HTTP-only cookies prevent XSS attacks (JavaScript cannot access token)
- Secure flag ensures cookies only transmitted over HTTPS in production
- SameSite=Strict prevents CSRF attacks
- Sliding expiration improves UX (active users stay logged in)
- Separate refresh token endpoint allows long-lived sessions without security risk

**Implementation Pattern**:
```python
# Cookie configuration
SESSION_COOKIE_NAME = "session_token"
SESSION_COOKIE_HTTPONLY = True
SESSION_COOKIE_SECURE = True  # Production only (HTTPS)
SESSION_COOKIE_SAMESITE = "strict"
SESSION_COOKIE_MAX_AGE = 30 * 24 * 60 * 60  # 30 days
SESSION_IDLE_TIMEOUT = 7 * 24 * 60 * 60  # 7 days idle logout

# Session validation with sliding window
async def get_session_from_cookie(request: Request):
    token = request.cookies.get(SESSION_COOKIE_NAME)
    if not token:
        return None

    session = await validate_session(token)
    if not session:
        return None

    # Update last_used_at (sliding window)
    await session.update(last_used_at=datetime.utcnow())
    return session
```

**Alternatives Considered**:
- **Local storage tokens**: Vulnerable to XSS, not recommended for sensitive auth tokens
- **Bearer tokens in Authorization header**: Requires client-side token management, more complex for web apps
- **JWT-only (no server-side session)**: Cannot revoke tokens, difficult to implement logout, limited security

### 6. Onboarding Flow Implementation

**Decision**: Implement onboarding as a separate flow step between authentication and application access, enforced via middleware/dependency injection.

**Rationale**:
- Clear separation between authentication (who you are) and onboarding (user profile)
- Middleware can check `onboarding_completed` flag and redirect as needed
- Frontend can query onboarding status to determine which page to show
- Onboarding can be mandatory (current requirement) or optional (future flexibility) with minimal code changes

**Implementation Pattern**:
```python
# Onboarding check dependency
async def require_onboarding(user: User = Depends(get_current_user)):
    if not user.onboarding_completed:
        raise HTTPException(
            status_code=403,
            detail="Onboarding required",
            headers={"Location": "/onboarding"}  # Hint for frontend redirect
        )
    return user

# Onboarding endpoints
@router.get("/auth/onboarding/status")
async def get_onboarding_status(user: User = Depends(get_current_user)):
    return {"completed": user.onboarding_completed}

@router.get("/auth/onboarding/options")
async def get_onboarding_options():
    return {
        "user_types": ["Student", "Researcher", "Teacher", "Engineer", "Other"],
        "areas_of_interest": ["Robotics", "AI/ML", "Computer Vision", "NLP", "Hardware"],
        "experience_levels": ["Beginner", "Intermediate", "Advanced"],
        "topics_of_interest": ["ROS2", "NVIDIA Isaac", "Voice Control", "LLM Integration", "Reinforcement Learning"]
    }

@router.post("/auth/onboarding/complete")
async def complete_onboarding(
    data: OnboardingData,
    user: User = Depends(get_current_user)
):
    # Validate and save onboarding profile
    profile = await create_onboarding_profile(user.id, data)

    # Update user flag
    user.onboarding_completed = True
    await user.save()

    return {"success": True, "redirect": "/"}  # Redirect to app
```

**Frontend Integration**:
```typescript
// Check onboarding status after login
const { data } = await fetch('/auth/onboarding/status');
if (!data.completed) {
    router.push('/onboarding');  // Redirect to onboarding page
} else {
    router.push('/');  // Redirect to app
}
```

**Alternatives Considered**:
- **Progressive onboarding**: Rejected for this iteration (out of scope), but architecture supports it
- **Optional onboarding**: Not current requirement, but easy to implement by changing middleware logic
- **Multi-step onboarding**: Deferred (non-goal), but can be added by tracking step progress

## Best Practices and Recommendations

### Security Checklist

- [x] **Password Hashing**: Use Argon2id (current implementation uses this via `password.py`)
- [x] **Session Token Hashing**: Hash tokens before storing in database (current implementation uses HMAC-SHA256)
- [x] **HTTP-only Cookies**: Prevent XSS attacks
- [x] **Secure Cookies**: HTTPS-only in production
- [x] **SameSite Cookies**: Prevent CSRF attacks
- [x] **CSRF Protection**: OAuth state parameter provides protection for OAuth flows
- [ ] **Email Verification**: Not implemented (out of scope for this iteration)
- [ ] **Password Reset**: Not implemented (out of scope)
- [ ] **Rate Limiting**: Not implemented (out of scope, but architecture must support future addition)
- [ ] **OAuth Token Encryption**: Access/refresh tokens currently stored in plain text (should encrypt)

### Production Configuration Checklist

- [ ] **Environment Variables**: All secrets in `.env`, never committed to git
- [ ] **CORS Configuration**: Whitelist specific frontend origin(s), not `*`
- [ ] **Cookie Configuration**: `secure=True`, `httpOnly=True`, `sameSite="strict"`
- [ ] **Database Connection**: Use connection pooling (asyncpg/psycopg3 default)
- [ ] **Error Handling**: Generic error messages to clients, detailed logs server-side
- [ ] **Logging**: Structured logging for auth events (login, logout, registration, failures)
- [ ] **Monitoring**: Track auth failure rates, session duration, onboarding completion rates

### Testing Strategy

1. **Unit Tests**:
   - Password hashing/verification
   - Session validation logic
   - Onboarding data validation
   - Token generation/verification

2. **Integration Tests**:
   - Full registration flow (email/password)
   - Full login flow
   - OAuth flow (Google/GitHub)
   - Onboarding flow
   - Session persistence across requests
   - Logout and session cleanup

3. **Contract Tests**:
   - API schema validation
   - Response format consistency
   - Error response structure

4. **End-to-End Tests** (with frontend):
   - Register → Auto-login → Onboarding → App access
   - Login → Onboarding (if not completed) → App access
   - OAuth → Account creation/linking → Onboarding → App access
   - Unauthenticated access blocked
   - Session expiration handling

## References

- **FastAPI Documentation**: https://fastapi.tiangolo.com/
- **Authlib (OAuth)**: https://docs.authlib.org/
- **Better Auth Patterns**: https://better-auth.com/ (conceptual patterns only, not Python implementation)
- **OWASP Authentication Cheat Sheet**: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
- **Argon2 Password Hashing**: https://password-hashing.net/argon2-specs.pdf

## Next Steps

**Phase 1**: Design & Contracts
1. Generate data-model.md with detailed entity definitions
2. Create OpenAPI contracts for authentication and onboarding endpoints
3. Generate quickstart.md for development setup
4. Update agent context with new technologies

**Phase 2**: Task Breakdown (via `/sp.tasks`)
1. Break down implementation into testable tasks
2. Prioritize tasks based on user stories
3. Define acceptance criteria for each task
