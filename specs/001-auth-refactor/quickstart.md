# Quickstart: Authentication Refactor & Integration

**Feature**: 001-auth-refactor
**Date**: 2025-12-16
**Phase**: Phase 1 - Design & Contracts

## Overview

This quickstart guide helps developers set up the development environment and understand the refactored authentication architecture. Follow these steps to run the unified backend locally with authentication and RAG chatbot functionality.

## Prerequisites

- Python 3.11 or higher
- Node.js 18+ (for Docusaurus frontend)
- PostgreSQL access (Neon Serverless Postgres)
- Git
- OAuth credentials (Google, GitHub)

## Architecture Overview

The refactored system runs a **single FastAPI application** serving both RAG chatbot and authentication endpoints:

```
Single FastAPI App (localhost:8000)
├── /api/*           → RAG Chatbot endpoints
├── /auth/*          → Authentication endpoints
└── /auth/onboarding/* → Onboarding endpoints
```

**Key Changes from Previous Structure**:
- ❌ **Before**: Separate servers for auth (`auth_backend/run.py`) and RAG (`backend/main.py`)
- ✅ **After**: Unified server in `rag-chatbot/backend/app/main.py` with both modules

## Environment Setup

### 1. Clone and Navigate

```bash
cd /path/to/AI-robotics
git checkout 001-auth-refactor
```

### 2. Install Backend Dependencies

```bash
cd rag-chatbot/backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

**Key Dependencies** (verify in `requirements.txt`):
- `fastapi>=0.115.0`
- `uvicorn[standard]>=0.30.0`
- `psycopg[binary]>=3.1.0`
- `authlib>=1.3.0`
- `argon2-cffi>=23.1.0`
- `pydantic>=2.0.0`

### 3. Configure Environment Variables

Create `.env` file in `rag-chatbot/backend/`:

```bash
# Database
DATABASE_URL=postgresql://user:password@endpoint.neon.tech/dbname

# Authentication
SESSION_SECRET=<generate-with-openssl-rand-hex-32>
COOKIE_SECURE=false  # Set to true in production (HTTPS)
COOKIE_SAMESITE=strict

# OAuth - Google
OAUTH_GOOGLE_CLIENT_ID=<your-google-client-id>
OAUTH_GOOGLE_CLIENT_SECRET=<your-google-client-secret>
OAUTH_GOOGLE_REDIRECT_URI=http://localhost:8000/auth/oauth/google/callback

# OAuth - GitHub
OAUTH_GITHUB_CLIENT_ID=<your-github-client-id>
OAUTH_GITHUB_CLIENT_SECRET=<your-github-client-secret>
OAUTH_GITHUB_REDIRECT_URI=http://localhost:8000/auth/oauth/github/callback

# Frontend
FRONTEND_URL=http://localhost:3000
CORS_ORIGINS=http://localhost:3000

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
```

**Generate SESSION_SECRET**:
```bash
python -c "import secrets; print(secrets.token_hex(32))"
```

### 4. Configure OAuth Applications

#### Google OAuth Setup:
1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Enable "Google+ API"
4. Create OAuth 2.0 credentials
5. Add authorized redirect URI: `http://localhost:8000/auth/oauth/google/callback`
6. Copy Client ID and Client Secret to `.env`

#### GitHub OAuth Setup:
1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Create new OAuth App
3. Set Authorization callback URL: `http://localhost:8000/auth/oauth/github/callback`
4. Copy Client ID and Client Secret to `.env`

### 5. Run Database Migrations

```bash
cd rag-chatbot/auth_backend
python -m migrations.001_initial_auth  # Existing auth schema
python -m migrations.002_onboarding    # New onboarding schema
```

**Manual Migration** (if automated scripts not available):
```bash
psql $DATABASE_URL < auth_backend/migrations/001_initial_auth.sql
psql $DATABASE_URL < auth_backend/migrations/002_onboarding.sql
```

### 6. Start the Unified Backend

```bash
cd rag-chatbot/backend
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 7. Verify Backend is Running

```bash
# Health check
curl http://localhost:8000/api/health

# Expected response:
# {"status": "healthy"}

# Check auth routes
curl http://localhost:8000/docs  # Swagger UI
```

You should see both RAG and Auth endpoints in the Swagger documentation.

### 8. Install and Run Frontend

```bash
cd AIdd-book
npm install
npm start
```

Frontend should start on `http://localhost:3000`.

## Testing the Refactored System

### Test 1: Registration Flow

1. Navigate to `http://localhost:3000/register`
2. Fill out registration form:
   - Email: `test@example.com`
   - Password: `SecurePassword123!`
   - Confirm Password: `SecurePassword123!`
3. Submit form
4. **Expected**: User is created and automatically logged in
5. **Expected**: Redirected to onboarding page (`/onboarding`)

### Test 2: Onboarding Flow

1. On onboarding page, fill out form:
   - User Type: `Researcher`
   - Area of Interest: `Robotics & Automation`
   - Experience Level: `Intermediate`
   - Topics: Select `ROS2`, `NVIDIA Isaac`
2. Submit form
3. **Expected**: Profile saved, `onboarding_completed = TRUE`
4. **Expected**: Redirected to base application URL (`/`)

### Test 3: Protected Chatbot Access

1. Navigate to chatbot page (`/chat`)
2. **Expected (if authenticated + onboarded)**: Chat interface loads
3. Send a test message
4. **Expected**: RAG backend processes message, returns response

### Test 4: Unauthenticated Access Blocked

1. Log out (click logout button)
2. Try to navigate directly to `/chat`
3. **Expected**: Redirected to `/login`

### Test 5: OAuth Flow (Google)

1. Navigate to `http://localhost:3000/login`
2. Click "Sign in with Google"
3. **Expected**: Redirected to Google OAuth consent screen
4. Authorize the application
5. **Expected**: Redirected back to app with session established
6. **If first-time user**: Redirected to onboarding page
7. **If returning user**: Redirected to app home

## API Endpoints Reference

### Authentication Endpoints

| Method | Path                         | Description              | Auth Required |
|--------|------------------------------|--------------------------|---------------|
| POST   | `/auth/register`             | Register new user        | No            |
| POST   | `/auth/login`                | Login user               | No            |
| POST   | `/auth/logout`               | Logout user              | Yes           |
| GET    | `/auth/me`                   | Get current user         | Yes           |
| POST   | `/auth/refresh`              | Refresh session token    | Yes           |
| GET    | `/auth/oauth/google`         | Initiate Google OAuth    | No            |
| GET    | `/auth/oauth/google/callback`| Google OAuth callback    | No            |
| GET    | `/auth/oauth/github`         | Initiate GitHub OAuth    | No            |
| GET    | `/auth/oauth/github/callback`| GitHub OAuth callback    | No            |

### Onboarding Endpoints

| Method | Path                          | Description              | Auth Required |
|--------|-------------------------------|--------------------------|---------------|
| GET    | `/auth/onboarding/status`     | Check onboarding status  | Yes           |
| GET    | `/auth/onboarding/options`    | Get form options         | No            |
| POST   | `/auth/onboarding/complete`   | Complete onboarding      | Yes           |
| GET    | `/auth/onboarding/profile`    | Get onboarding profile   | Yes           |

### RAG Chatbot Endpoints (Existing - Now Auth Protected)

| Method | Path                    | Description              | Auth Required |
|--------|-------------------------|--------------------------|---------------|
| GET    | `/api/health`           | Health check             | No            |
| POST   | `/api/chat`             | Send chat message        | Yes (+ onboarding) |
| GET    | `/api/sessions`         | Get chat sessions        | Yes (+ onboarding) |
| POST   | `/api/sessions`         | Create chat session      | Yes (+ onboarding) |
| DELETE | `/api/sessions/{id}`    | Delete chat session      | Yes (+ onboarding) |

## Development Workflow

### Making Changes to Auth Backend

1. Edit files in `rag-chatbot/auth_backend/`
2. Backend auto-reloads (uvicorn `--reload` flag)
3. Test changes via Swagger UI or frontend
4. Write tests in `auth_backend/tests/`

### Making Changes to RAG Backend

1. Edit files in `rag-chatbot/backend/app/`
2. Backend auto-reloads
3. Ensure no regression in existing functionality
4. Run integration tests

### Adding New Endpoints

1. Create route handler in appropriate module:
   - Auth: `auth_backend/api/routes/`
   - RAG: `backend/app/api/`
2. Register router in `backend/app/main.py`:
   ```python
   app.include_router(new_router, prefix="/api", tags=["NewFeature"])
   ```
3. Update OpenAPI contracts in `specs/001-auth-refactor/contracts/`
4. Write tests

## Troubleshooting

### Issue: OAuth Redirect Mismatch

**Symptom**: OAuth fails with "redirect_uri_mismatch" error

**Solution**: Ensure OAuth provider's authorized redirect URI exactly matches:
- Dev: `http://localhost:8000/auth/oauth/{provider}/callback`
- No trailing slash
- Correct port (8000)

### Issue: CORS Errors

**Symptom**: Frontend requests blocked by CORS policy

**Solution**: Check CORS configuration in `backend/app/main.py`:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Must match frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Issue: Database Connection Failures

**Symptom**: "Connection refused" or "authentication failed"

**Solution**:
1. Verify `DATABASE_URL` in `.env`
2. Check Neon database is accessible
3. Ensure database has required tables (run migrations)

### Issue: Onboarding Not Enforced

**Symptom**: Can access chatbot without completing onboarding

**Solution**: Ensure `get_current_user_with_onboarding` dependency is used:
```python
@router.post("/chat")
async def chat(user: User = Depends(get_current_user_with_onboarding)):
    # This enforces onboarding check
    pass
```

### Issue: Session Not Persisting

**Symptom**: User logged out on page refresh

**Solution**:
1. Check cookie configuration (`HttpOnly`, `Secure`, `SameSite`)
2. Verify frontend sends cookies with requests (`credentials: 'include'`)
3. Ensure `SESSION_SECRET` is set and consistent

## Next Steps

1. **Run `/sp.tasks`**: Break down implementation into testable tasks
2. **Implement Tasks**: Follow task-driven development
3. **Write Tests**: Unit, integration, and E2E tests
4. **Deploy**: Configure production environment variables

## Resources

- **API Documentation**: http://localhost:8000/docs (Swagger UI)
- **Feature Spec**: [spec.md](./spec.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contracts**: [contracts/](./contracts/)
- **Research**: [research.md](./research.md)
