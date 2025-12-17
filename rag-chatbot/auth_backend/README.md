# Authentication Backend

Better Auth-based authentication system for the RAG Chatbot, implemented in FastAPI with Neon Serverless Postgres.

## Features

- ✅ **Email/Password Authentication**: Secure signup and login with Argon2id password hashing
- ✅ **Google OAuth 2.0**: Social login with account linking
- ✅ **GitHub OAuth 2.0**: Social login with account linking
- ✅ **Mandatory Onboarding**: User profile collection after registration/login
- ✅ **Session Management**: Database-backed sessions with HMAC-SHA256 token hashing
- ✅ **Cookie-based Auth**: HttpOnly, Secure, SameSite cookies
- ✅ **Protected Endpoints**: FastAPI dependency injection for authentication
- ✅ **Account Linking**: Automatically links OAuth accounts to existing email accounts

## Quick Start

### 1. Installation

```bash
cd auth_backend
pip install -r requirements.txt
```

### 2. Configuration

Copy `.env.example` to `.env` and update:

```bash
cp .env.example .env
```

**Required settings:**
- `DATABASE_URL`: Your Neon Postgres connection string (same as RAG chatbot)
- `SESSION_SECRET`: Generate with `python -c "import secrets; print(secrets.token_urlsafe(32))"`

**Optional settings:**
- `GOOGLE_CLIENT_ID` and `GOOGLE_CLIENT_SECRET`: For OAuth (get from Google Cloud Console)

### 3. Database Migration

Run the migration to create auth tables:

```bash
# Using Alembic (if configured)
alembic upgrade head

# Or apply the migration manually
python -c "from auth_backend.migrations.001_initial_auth import upgrade; upgrade()"
```

### 4. Run the Server

```bash
uvicorn auth_backend.main:app --reload --port 8000
```

Visit http://localhost:8000/api/auth/docs for interactive API documentation.

## API Endpoints

### Authentication

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `/auth/register` | POST | Create new account | No |
| `/auth/login` | POST | Log in with email/password | No |
| `/auth/logout` | POST | Log out and clear session | Yes |
| `/auth/me` | GET | Get current user info | Yes |
| `/auth/refresh` | POST | Refresh session token | Yes |

### OAuth

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `/auth/oauth/google` | GET | Initiate Google OAuth | No |
| `/auth/oauth/google/callback` | GET | Google OAuth callback | No |
| `/auth/oauth/github` | GET | Initiate GitHub OAuth | No |
| `/auth/oauth/github/callback` | GET | GitHub OAuth callback | No |

### Onboarding

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `/auth/onboarding/options` | GET | Get predefined options | No |
| `/auth/onboarding/complete` | POST | Complete onboarding | Yes |
| `/auth/onboarding/profile` | GET | Get user's profile | Yes |
| `/auth/onboarding/profile` | PUT | Update user's profile | Yes |
| `/auth/onboarding/status` | GET | Check onboarding status | Yes |

## Usage Examples

### Register a New User

```bash
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{
    "email": "alice@example.com",
    "name": "Alice Smith",
    "password": "SecurePass123"
  }'
```

### Login

```bash
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{
    "email": "alice@example.com",
    "password": "SecurePass123"
  }' \
  -c cookies.txt
```

### Access Protected Route

```bash
curl -X GET http://localhost:8000/api/auth/session \
  -b cookies.txt
```

### Logout

```bash
curl -X POST http://localhost:8000/api/auth/logout \
  -b cookies.txt
```

## Integration with Main App

To integrate with your main RAG chatbot application:

### 1. Import the Auth Dependency

```python
from fastapi import Depends
from auth_backend.api.deps import get_current_user, get_current_user_with_onboarding
from auth_backend.models.user import User

# For endpoints that require authentication only
@app.get("/api/profile")
async def profile_endpoint(
    user: User = Depends(get_current_user)
):
    return {"message": f"Hello {user.name}!"}

# For endpoints that require completed onboarding
@app.get("/api/chat")
async def chat_endpoint(
    user: User = Depends(get_current_user_with_onboarding)
):
    return {"message": f"Hello {user.name}! You can access the chatbot."}
```

### 2. Mount the Auth Router

```python
from fastapi import FastAPI
from auth_backend.api.routes import auth, oauth, onboarding

app = FastAPI()
app.include_router(auth.router, prefix="/auth")
app.include_router(oauth.router, prefix="/auth/oauth")
app.include_router(onboarding.router, prefix="/auth")
```

### 3. Share the Database Connection

Both apps should use the same `DATABASE_URL` to share the auth tables.

## Frontend Integration Guide

### Authentication Flow

#### 1. Register/Login Response

After successful registration or login, the API returns:

```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "User Name",
    "onboarding_completed": false
  },
  "session": {
    "id": "uuid",
    "expires_at": "2025-02-15T10:00:00Z"
  },
  "message": "Logged in successfully",
  "onboarding_required": true
}
```

**Check `onboarding_required` field:**
- If `true`: Redirect to onboarding page (`/onboarding`)
- If `false`: Redirect to main app

#### 2. Onboarding Flow

**Step 1: Fetch available options**

```typescript
const response = await fetch('http://localhost:8000/auth/onboarding/options');
const options = await response.json();
// {
//   "user_types": ["student", "educator", "researcher", "hobbyist", "professional"],
//   "areas_of_interest": ["robotics", "ai_ml", "iot", "automation", "research"],
//   "experience_levels": ["beginner", "intermediate", "advanced"],
//   "topics_of_interest": ["ros2", "computer_vision", "nlp", ...]
// }
```

**Step 2: Submit onboarding data**

```typescript
const response = await fetch('http://localhost:8000/auth/onboarding/complete', {
  method: 'POST',
  credentials: 'include',  // Include session cookie
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    user_type: 'student',
    area_of_interest: 'robotics',
    experience_level: 'beginner',
    topics_of_interest: ['ros2', 'simulation']
  })
});
```

After successful onboarding, redirect to main app.

#### 3. Protected Routes

For routes that require completed onboarding (like chatbot access):

```typescript
// Check onboarding status
const checkOnboarding = async () => {
  const response = await fetch('http://localhost:8000/auth/onboarding/status', {
    credentials: 'include'
  });
  const { onboarding_completed } = await response.json();

  if (!onboarding_completed) {
    // Redirect to onboarding
    window.location.href = '/onboarding';
  }
};
```

#### 4. Session Management

**Check current user:**

```typescript
const getCurrentUser = async () => {
  const response = await fetch('http://localhost:8000/auth/me', {
    credentials: 'include'
  });

  if (response.status === 401) {
    // Not authenticated - redirect to login
    window.location.href = '/login';
    return null;
  }

  return await response.json();
};
```

**Refresh session:**

```typescript
const refreshSession = async () => {
  await fetch('http://localhost:8000/auth/refresh', {
    method: 'POST',
    credentials: 'include'
  });
};
```

**Logout:**

```typescript
const logout = async () => {
  await fetch('http://localhost:8000/auth/logout', {
    method: 'POST',
    credentials: 'include'
  });
  window.location.href = '/login';
};
```

#### 5. OAuth Flow

**Google OAuth:**

```typescript
// Redirect to OAuth initiation
window.location.href = 'http://localhost:8000/auth/oauth/google';

// After callback, user is redirected to frontend_url with session cookie set
// Check onboarding_required in the AuthResponse to determine next step
```

**GitHub OAuth:**

```typescript
// Redirect to OAuth initiation
window.location.href = 'http://localhost:8000/auth/oauth/github';
```

### Error Handling

```typescript
const handleAuthError = (response: Response) => {
  if (response.status === 401) {
    // Not authenticated
    window.location.href = '/login';
  } else if (response.status === 403) {
    // Forbidden - likely onboarding required
    window.location.href = '/onboarding';
  } else if (response.status === 409) {
    // Conflict - email already exists
    alert('Email already registered');
  }
};
```

### CORS Configuration

Ensure your frontend URL is in the backend's CORS origins:

```env
# backend/.env
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

### Cookie Configuration

For development (HTTP):
```env
COOKIE_SECURE=false
COOKIE_SAMESITE=lax
```

For production (HTTPS):
```env
COOKIE_SECURE=true
COOKIE_SAMESITE=strict
```

## Database Schema

### Users Table

- `id` (UUID): Primary key
- `email` (String): Unique, case-insensitive
- `name` (String): Display name
- `password_hash` (Text): Argon2id hash (nullable for OAuth users)
- `onboarding_completed` (Boolean): Whether user completed onboarding
- `email_verified_at` (Timestamp): Email verification timestamp
- `created_at`, `updated_at` (Timestamps)

### Sessions Table

- `id` (UUID): Primary key
- `user_id` (UUID): Foreign key to users
- `token_hash` (String): HMAC-SHA256 hashed session token
- `last_used_at` (Timestamp): For sliding expiration
- `expires_at` (Timestamp): Absolute expiration (30 days)
- `revoked` (Boolean): Manual revocation flag
- `ip_address`, `user_agent` (String): Audit info

### OAuth Accounts Table

- `id` (UUID): Primary key
- `user_id` (UUID): Foreign key to users
- `provider` (String): OAuth provider (e.g., "google", "github")
- `provider_account_id` (String): Provider's user ID
- `access_token`, `refresh_token` (Text): OAuth tokens
- `expires_at` (Timestamp): Token expiration

### Onboarding Profiles Table

- `id` (UUID): Primary key
- `user_id` (UUID): Foreign key to users (unique)
- `user_type` (String): Type of user (student, educator, researcher, hobbyist, professional)
- `area_of_interest` (String): Primary area (robotics, ai_ml, iot, automation, research)
- `experience_level` (String): Experience (beginner, intermediate, advanced)
- `topics_of_interest` (JSONB): Array of specific topics (optional)
- `created_at`, `updated_at` (Timestamps)

## Security Features

✅ **Password Security**
- Argon2id hashing (64MB memory, 3 iterations)
- Minimum 8-character passwords
- Constant-time password verification

✅ **Session Security**
- HMAC-SHA256 token hashing
- HttpOnly cookies (not accessible to JavaScript)
- Secure flag in production (HTTPS only)
- SameSite=Lax (CSRF protection)
- 30-day absolute expiration
- 7-day sliding idle timeout

✅ **OAuth Security**
- State parameter for CSRF protection
- Account linking by email
- Secure token storage

✅ **API Security**
- Generic error messages (prevents email enumeration)
- Rate limiting ready (add via middleware)
- CORS configured

## Development

### Running Tests

```bash
pytest auth_backend/tests/
```

### Linting

```bash
ruff check auth_backend/
ruff format auth_backend/
```

### Database Migrations

Create a new migration:

```bash
alembic revision -m "description"
```

Apply migrations:

```bash
alembic upgrade head
```

Rollback:

```bash
alembic downgrade -1
```

## Configuration Reference

See `.env.example` for all available configuration options.

## License

Part of the AI-Robotics Textbook project.

## Support

For issues or questions, see the main project repository.
