# Better Auth Research: FastAPI Integration Best Practices

## Document Information
- **Date**: 2025-12-16
- **Purpose**: Research Better Auth patterns for FastAPI backend authentication refactor
- **Status**: Research Complete

## Executive Summary

Better Auth is a TypeScript/JavaScript authentication library primarily designed for Next.js and other JavaScript frameworks. **It is NOT natively compatible with Python/FastAPI**. Your current implementation is a **custom Python authentication system inspired by Better Auth patterns**, not an actual Better Auth integration.

## Key Finding: Better Auth is NOT Available for FastAPI

**CRITICAL**: Better Auth is a Node.js/TypeScript library. There is no official Better Auth SDK for Python or FastAPI. What you have is a custom-built authentication system that follows Better Auth's conceptual patterns.

Your implementation:
- Uses Better Auth's **database schema conventions** (users, sessions, oauth_accounts tables)
- Follows Better Auth's **security patterns** (HTTP-only cookies, token hashing, session management)
- Implements Better Auth's **OAuth flow concepts** (state validation, account linking)

But it's **entirely custom Python/FastAPI code**, not an integration with Better Auth itself.

## 1. Current Implementation Analysis

### Architecture Overview

```
/rag-chatbot/backend/
├── app/
│   └── main.py              # FastAPI app with /api/* routes (RAG chatbot)
└── auth_backend/            # Custom auth system (Better Auth-inspired)
    ├── models/              # SQLAlchemy models
    │   ├── user.py          # User table
    │   ├── session.py       # Session table
    │   └── oauth_account.py # OAuth accounts table
    ├── services/
    │   ├── session.py       # Session management
    │   ├── oauth.py         # OAuth flows (Google)
    │   └── password.py      # Password hashing
    └── api/routes/
        ├── auth.py          # /api/auth/* endpoints
        └── oauth.py         # /api/auth/oauth/* endpoints
```

### Current Route Structure

- **Public routes**: `/api/health`, `/api/auth/login`, `/api/auth/register`, `/api/auth/oauth/*`
- **Protected routes**: `/api/*` (RAG chatbot endpoints - should require authentication)
- **OAuth callback**: `/api/auth/oauth/google/callback`

## 2. Database Schema (Better Auth Convention)

Your current schema follows Better Auth's conventions correctly:

### Users Table
```python
class User(Base, TimestampMixin):
    __tablename__ = "users"

    id: UUID                      # Primary key
    email: str                    # Unique, indexed
    name: str                     # Display name
    password_hash: Optional[str]  # Null for OAuth-only users
    email_verified_at: Optional[DateTime]
    created_at: DateTime
    updated_at: DateTime
```

### Sessions Table
```python
class Session(Base, TimestampMixin):
    __tablename__ = "auth_sessions"

    id: UUID
    user_id: UUID                 # FK to users
    token_hash: str               # HMAC-SHA256 hashed token (unique, indexed)
    last_used_at: DateTime        # For sliding expiration
    expires_at: DateTime          # Absolute expiration (30 days)
    revoked: bool
    ip_address: Optional[str]
    user_agent: Optional[str]
    created_at: DateTime
    updated_at: DateTime
```

### OAuth Accounts Table
```python
class OAuthAccount(Base, TimestampMixin):
    __tablename__ = "oauth_accounts"

    id: UUID
    user_id: UUID                 # FK to users
    provider: str                 # 'google', 'github', etc.
    provider_account_id: str      # User's ID from provider
    access_token: Optional[str]   # OAuth access token
    refresh_token: Optional[str]  # OAuth refresh token
    expires_at: Optional[DateTime]
    scope: Optional[str]
    token_type: Optional[str]     # Usually 'Bearer'
    created_at: DateTime
    updated_at: DateTime
```

## 3. Session Management (Better Auth Patterns)

### Your Current Implementation ✅ CORRECT

```python
# Session creation (session.py:32-76)
async def create_session(self, db, user_id, ip_address, user_agent):
    # Generate random token + HMAC hash
    plain_token, hashed_token = generate_and_hash_token(self.secret_key)

    # Store hashed token in database
    session = Session(
        user_id=user_id,
        token_hash=hashed_token,
        expires_at=datetime.utcnow() + timedelta(days=30),  # 30-day absolute expiration
        last_used_at=datetime.utcnow()
    )

    # Return plain token to set in HTTP-only cookie
    return session, plain_token
```

### Cookie Configuration ✅ CORRECT

```python
# From auth.py:91-98 and oauth.py:124-131
response.set_cookie(
    key="session_token",          # Cookie name
    value=plain_token,            # Plain token (not hash!)
    httponly=True,                # ✅ Prevent JavaScript access
    secure=settings.secure_cookies,  # ✅ HTTPS-only in production
    samesite="lax",               # ✅ CSRF protection
    max_age=30 * 24 * 60 * 60     # ✅ 30 days
)
```

### Token Hashing ✅ CORRECT

Your implementation uses **HMAC-SHA256** with a secret key (similar to Better Auth's approach):

```python
# From utils/security.py
def generate_and_hash_token(secret_key: str) -> Tuple[str, str]:
    plain_token = secrets.token_urlsafe(32)  # Random token
    hashed_token = hash_token(plain_token, secret_key)
    return plain_token, hashed_token

def hash_token(token: str, secret_key: str) -> str:
    return hmac.new(
        secret_key.encode(),
        token.encode(),
        hashlib.sha256
    ).hexdigest()
```

### Session Validation Pattern ✅ CORRECT

```python
# From deps.py:14-55
async def get_current_user(
    db: AsyncSession = Depends(get_db),
    session_token: Optional[str] = Cookie(None, alias="session_token")
) -> User:
    if not session_token:
        raise HTTPException(401, "Not authenticated")

    # Hash the token and query database
    result = await session_service.validate_session(db, session_token)
    if not result:
        raise HTTPException(401, "Session invalid or expired")

    session, user = result
    return user
```

## 4. OAuth Configuration (Google & GitHub)

### Current Google OAuth Implementation

```python
# From services/oauth.py:24-34
class OAuthService:
    def __init__(self):
        self.oauth = OAuth()
        if settings.google_client_id and settings.google_client_secret:
            self.oauth.register(
                name="google",
                client_id=settings.google_client_id,
                client_secret=settings.google_client_secret,
                server_metadata_url="https://accounts.google.com/.well-known/openid-configuration",
                client_kwargs={"scope": "openid email profile"}
            )
```

### Redirect URL Configuration

**Development**: `http://localhost:8000/api/auth/oauth/google/callback`
**Production**: `https://your-domain.com/api/auth/oauth/google/callback`

### OAuth Flow Pattern ✅ CORRECT

1. **Initiate OAuth** (`/api/auth/oauth/google`):
   ```python
   # Generate CSRF state
   state = oauth_service.generate_state()

   # Redirect to Google
   return await oauth_service.oauth.google.authorize_redirect(
       request, redirect_uri, state=state
   )
   ```

2. **Handle Callback** (`/api/auth/oauth/google/callback`):
   ```python
   # Exchange code for token
   token = await oauth_service.oauth.google.authorize_access_token(request)

   # Get user info
   user_info = token.get("userinfo")

   # Find or create user (account linking)
   user, oauth_account, is_new_user = await oauth_service.find_or_create_user_from_oauth(
       db, "google", user_info
   )

   # Create session
   session, plain_token = await session_service.create_session(db, user.id)

   # Set cookie and redirect to frontend
   response.set_cookie("session_token", plain_token, httponly=True)
   return RedirectResponse(url=settings.frontend_url)
   ```

### Account Linking Pattern ✅ CORRECT

```python
# From services/oauth.py:53-139
async def find_or_create_user_from_oauth(self, db, provider, user_info):
    provider_account_id = user_info.get("sub")
    email = user_info.get("email", "").lower()

    # Check if OAuth account exists
    oauth_account = await db.query(OAuthAccount).filter_by(
        provider=provider,
        provider_account_id=provider_account_id
    ).first()

    if oauth_account:
        # Existing OAuth account - return user
        return user, oauth_account, False

    # Check if user with this email exists (account linking)
    user = await db.query(User).filter_by(email=email).first()

    is_new_user = user is None
    if not user:
        # Create new user
        user = User(email=email, name=user_info.get("name"), password_hash=None)
        db.add(user)
        await db.commit()

    # Create OAuth account record
    oauth_account = OAuthAccount(
        user_id=user.id,
        provider=provider,
        provider_account_id=provider_account_id
    )
    db.add(oauth_account)
    await db.commit()

    return user, oauth_account, is_new_user
```

## 5. Middleware Patterns for Route Protection

### Current Status ❌ INCOMPLETE

You have authentication dependencies but **no global middleware** to protect `/api/*` routes.

### Recommended Implementation

**Option 1: FastAPI Dependency with Exclusions** (Recommended)

```python
# In app/main.py
from fastapi import Depends, Request, FastAPI
from typing import Optional

# Public route patterns
PUBLIC_ROUTES = {
    "/",
    "/docs",
    "/openapi.json",
    "/api/health",
    "/api/auth/login",
    "/api/auth/register",
    "/api/auth/oauth/google",
    "/api/auth/oauth/google/callback",
}

def is_public_route(path: str) -> bool:
    """Check if route should bypass authentication."""
    return any(path.startswith(route) for route in PUBLIC_ROUTES)

async def auth_middleware(
    request: Request,
    user: Optional[User] = Depends(get_current_user_optional)
):
    """Global authentication check for protected routes."""
    if is_public_route(request.url.path):
        return None  # Skip auth check

    if not user:
        raise HTTPException(401, "Authentication required")

    return user

# Apply to all /api/* routes
app = FastAPI(dependencies=[Depends(auth_middleware)])
```

**Option 2: Explicit Per-Route Dependencies**

```python
# In each protected route file
from fastapi import Depends, APIRouter
from auth_backend.api.deps import get_current_user

router = APIRouter()

@router.get("/api/conversations", dependencies=[Depends(get_current_user)])
async def get_conversations(user: User = Depends(get_current_user)):
    # User is guaranteed to be authenticated
    return {"user_id": user.id}
```

## 6. Custom User Data Extension

### Current Schema Support ✅ READY

Your User model can be extended with custom fields:

```python
# In models/user.py
class User(Base, TimestampMixin):
    __tablename__ = "users"

    # Core fields
    id: UUID
    email: str
    name: str
    password_hash: Optional[str]

    # Custom fields (add these)
    onboarding_completed: Mapped[bool] = mapped_column(
        Boolean, default=False, nullable=False
    )
```

### Create Related Onboarding Table

```python
# New file: models/onboarding_profile.py
class OnboardingProfile(Base, TimestampMixin):
    __tablename__ = "onboarding_profiles"

    id: Mapped[uuid.UUID] = mapped_column(UUID(as_uuid=True), primary_key=True)
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        nullable=False,
        unique=True  # One profile per user
    )

    # Onboarding data
    role: Mapped[Optional[str]]
    interests: Mapped[Optional[str]]  # JSON or comma-separated
    completed_steps: Mapped[int] = mapped_column(default=0)

    # Relationship
    user: Mapped["User"] = relationship("User", back_populates="onboarding_profile")
```

### Migration Required

```bash
# Generate migration with Alembic
cd /mnt/d/AIDD-hackathon-book/AI-robotics/rag-chatbot/backend
alembic revision --autogenerate -m "Add onboarding fields"
alembic upgrade head
```

## 7. Security Best Practices Checklist

### ✅ Currently Implemented Correctly

- [x] **HTTP-only cookies**: Prevents XSS attacks
- [x] **Token hashing**: HMAC-SHA256 before database storage
- [x] **Secure flag**: Enabled in production (HTTPS)
- [x] **SameSite=lax**: CSRF protection
- [x] **Password hashing**: Using Argon2id (from password.py)
- [x] **Session expiration**: 30-day absolute, sliding window via last_used_at
- [x] **OAuth state validation**: CSRF protection in OAuth flows

### ❌ Missing or Needs Improvement

- [ ] **Global route protection**: Need middleware for `/api/*`
- [ ] **CORS configuration**: Ensure only trusted origins
- [ ] **Rate limiting**: Missing from auth endpoints
- [ ] **Email verification**: Not implemented for email/password signup
- [ ] **Password reset flow**: Not implemented
- [ ] **Session revocation**: No endpoint to revoke all sessions
- [ ] **OAuth token encryption**: Access/refresh tokens stored in plain text

## 8. Production Configuration Checklist

### Environment Variables (.env)

```bash
# Database
DATABASE_URL=postgresql+psycopg://user:password@host/database?sslmode=require

# Authentication
SESSION_SECRET=<64-character-random-hex-string>  # CRITICAL: Change in production!
SESSION_COOKIE_NAME=session_token
SESSION_MAX_AGE_DAYS=30
SESSION_IDLE_TIMEOUT_DAYS=7

# Security
SECURE_COOKIES=true           # HTTPS-only cookies
SAME_SITE_COOKIES=lax         # or 'strict' for higher security

# CORS
CORS_ORIGINS=https://your-domain.com

# Google OAuth
GOOGLE_CLIENT_ID=your-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-client-secret
GOOGLE_REDIRECT_URI=https://your-domain.com/api/auth/oauth/google/callback

# GitHub OAuth (if adding)
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
GITHUB_REDIRECT_URI=https://your-domain.com/api/auth/oauth/github/callback

# Frontend
FRONTEND_URL=https://your-domain.com
```

### Security Configuration

```python
# In config.py for production
secure_cookies: bool = True              # Force HTTPS-only cookies
same_site_cookies: str = "strict"        # Maximum CSRF protection
session_secret: str = os.urandom(32).hex()  # 256-bit secret
```

## 9. Potential Pitfalls & Solutions

### Pitfall 1: Mixing Auth and API Ports

**Problem**: Running auth on `localhost:8000/api/auth/*` and frontend on `localhost:3000` can cause cookie issues.

**Solution**:
- Keep all backend routes on single port (8000)
- Ensure `CORS_ORIGINS` includes frontend origin
- Use `credentials: 'include'` in frontend fetch calls

### Pitfall 2: OAuth Redirect URL Mismatch

**Problem**: Google/GitHub OAuth callback fails with "redirect_uri_mismatch"

**Solution**:
- Exact match required in OAuth provider console
- Development: `http://localhost:8000/api/auth/oauth/google/callback`
- Production: `https://your-domain.com/api/auth/oauth/google/callback`
- Update `.env` file when deploying

### Pitfall 3: Session Token Not Sent to Backend

**Problem**: Frontend makes requests but session cookie not included

**Solution**:
```typescript
// Frontend fetch calls must include credentials
fetch('http://localhost:8000/api/conversations', {
  credentials: 'include'  // Critical for cookie transmission
})
```

### Pitfall 4: Custom Fields Not in Database

**Problem**: Adding `user.onboarding_completed` without migration causes errors

**Solution**:
```bash
# Always run migrations after model changes
alembic revision --autogenerate -m "Add custom fields"
alembic upgrade head
```

### Pitfall 5: OAuth Access Tokens Stored Unencrypted

**Problem**: Sensitive tokens in plain text in database

**Solution** (from Better Auth docs):
```python
# Use database hooks to encrypt before storage
def encrypt_token(token: str) -> str:
    # Use Fernet or similar encryption
    from cryptography.fernet import Fernet
    cipher = Fernet(encryption_key)
    return cipher.encrypt(token.encode()).decode()

# In services/oauth.py
oauth_account.access_token = encrypt_token(token["access_token"])
```

## 10. Next Steps for Implementation

### Phase 1: Complete Core Authentication
1. Add global route protection middleware
2. Implement email verification flow
3. Add password reset functionality
4. Add session revocation endpoints

### Phase 2: Enhance OAuth
1. Add GitHub OAuth provider
2. Implement token encryption
3. Add OAuth token refresh logic

### Phase 3: User Management
1. Add custom user fields (onboarding_completed)
2. Create onboarding_profiles table
3. Implement user profile update endpoints

### Phase 4: Production Hardening
1. Add rate limiting (use slowapi or similar)
2. Implement CSRF token validation
3. Add security headers middleware
4. Set up monitoring and logging

## 11. Key Takeaways

1. **Better Auth is NOT available for Python/FastAPI** - Your implementation is custom Python code following Better Auth patterns

2. **Current implementation is mostly correct** - Session management, OAuth flows, and database schema follow best practices

3. **Missing pieces**:
   - Global route protection
   - Email verification
   - Password reset
   - OAuth token encryption

4. **Security is solid** for cookies and session management but needs:
   - Rate limiting
   - CSRF tokens for non-OAuth endpoints
   - OAuth token encryption

5. **Schema follows Better Auth conventions** - Easy to reference Better Auth docs for patterns even though it's custom Python code

## References

- Better Auth Documentation (for patterns only - not actual integration)
- FastAPI Security: https://fastapi.tiangolo.com/tutorial/security/
- Authlib (your OAuth library): https://docs.authlib.org/en/latest/
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
