# Quickstart Guide: Better Auth Development Setup

**Feature**: 001-better-auth
**Date**: 2025-12-14
**Phase**: 1 (Development Environment Setup)

## Overview

This guide provides step-by-step instructions for setting up the local development environment for the Better Auth integration with FastAPI and Neon Serverless Postgres.

---

## Prerequisites

- **Python**: 3.11 or higher
- **Node.js**: 18.x or higher (for Docusaurus frontend)
- **Git**: Latest version
- **Google Cloud Console**: Access to create OAuth 2.0 credentials
- **Neon Account**: Free tier account at [neon.tech](https://neon.tech)
- **Redis** (optional): Upstash free tier or local Redis for rate limiting

---

## 1. Environment Setup

### 1.1. Clone Repository

```bash
git clone <repository-url>
cd AI-robotics
git checkout 001-better-auth
```

### 1.2. Backend Setup (FastAPI)

#### Create Python Virtual Environment

```bash
cd backend
python3.11 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

#### Install Dependencies

Create `backend/requirements.txt`:
```txt
# Web Framework
fastapi[all]==0.115.0
uvicorn[standard]==0.30.0

# Database
sqlalchemy[asyncio]==2.0.35
alembic==1.13.2
asyncpg==0.29.0
psycopg2-binary==2.9.9  # For Alembic

# Authentication
argon2-cffi==23.1.0
pyjwt[crypto]==2.9.0

# OAuth
authlib==1.3.1
httpx==0.27.0

# Validation
pydantic[email]==2.9.2
pydantic-settings==2.5.2

# Rate Limiting
slowapi==0.1.9
redis==5.0.8

# CORS
python-multipart==0.0.9

# Environment
python-dotenv==1.0.1

# Development
pytest==8.3.3
pytest-asyncio==0.24.0
httpx==0.27.0  # For testing
faker==30.3.0  # For test data
```

Install packages:
```bash
pip install -r requirements.txt
```

---

## 2. Database Setup (Neon)

### 2.1. Create Neon Project

1. Go to [Neon Console](https://console.neon.tech)
2. Click "Create Project"
3. Name: `better-auth-dev`
4. Region: Choose closest to your location
5. Postgres Version: 15 or higher
6. Click "Create"

### 2.2. Get Connection String

1. In Neon Console, go to your project dashboard
2. Copy the connection string (it will look like):
   ```
   postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```
3. Save this for the next step

### 2.3. Create `.env` File

Create `backend/.env`:
```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:password@ep-xyz.us-east-2.aws.neon.tech/neondb?sslmode=require
NEON_CONNECTION_STRING=postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/neondb?sslmode=require

# Authentication
SESSION_COOKIE_NAME=session
SESSION_SECRET_PEPPER=your-64-char-random-hex-string-here
SESSION_MAX_AGE_DAYS=30
SESSION_IDLE_TIMEOUT_DAYS=7

# Google OAuth (will be configured in step 3)
GOOGLE_CLIENT_ID=your-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-client-secret
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/oauth/google/callback

# Security
SECURE_COOKIES=false  # Set to true in production
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000

# Rate Limiting (optional - use Redis URL if available)
REDIS_URL=redis://localhost:6379
RATE_LIMIT_ENABLED=true

# Logging
LOG_LEVEL=DEBUG
AUDIT_LOG_ENABLED=true
```

**Generate SESSION_SECRET_PEPPER**:
```bash
python3 -c "import secrets; print(secrets.token_hex(32))"
```

---

## 3. Google OAuth Setup

### 3.1. Create Google Cloud Project

1. Go to [Google Cloud Console](https://console.cloud.google.com)
2. Create a new project or select existing one
3. Name: `better-auth-dev`

### 3.2. Enable Google+ API

1. In Cloud Console, go to "APIs & Services" → "Library"
2. Search for "Google+ API"
3. Click "Enable"

### 3.3. Create OAuth 2.0 Credentials

1. Go to "APIs & Services" → "Credentials"
2. Click "Create Credentials" → "OAuth client ID"
3. Application type: "Web application"
4. Name: `Better Auth Dev`
5. Authorized JavaScript origins:
   - `http://localhost:8000`
   - `http://localhost:3000`
6. Authorized redirect URIs:
   - `http://localhost:8000/api/auth/oauth/google/callback`
7. Click "Create"
8. Copy **Client ID** and **Client Secret**
9. Update `backend/.env` with these values

### 3.4. Configure OAuth Consent Screen

1. Go to "APIs & Services" → "OAuth consent screen"
2. User Type: "External"
3. App name: `Better Auth Dev`
4. User support email: Your email
5. Scopes: Add `openid`, `email`, `profile`
6. Test users: Add your email for testing
7. Save and continue

---

## 4. Database Migrations

### 4.1. Initialize Alembic

```bash
cd backend
alembic init migrations
```

### 4.2. Configure Alembic

Edit `backend/alembic.ini`:
```ini
# Replace this line:
# sqlalchemy.url = driver://user:pass@localhost/dbname

# With (leave empty, we'll use env.py):
# sqlalchemy.url =
```

Edit `backend/migrations/env.py`:
```python
import os
import sys
from logging.config import fileConfig
from sqlalchemy import engine_from_config, pool
from alembic import context
from dotenv import load_dotenv

# Add backend to path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Load environment variables
load_dotenv()

# Import models
from src.models import Base

# Alembic Config object
config = context.config

# Set database URL from environment
config.set_main_option("sqlalchemy.url", os.getenv("DATABASE_URL"))

# Interpret the config file for Python logging
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

# Set metadata for autogenerate
target_metadata = Base.metadata

# ... rest of env.py (keep default async configuration)
```

### 4.3. Create Initial Migration

```bash
alembic revision --autogenerate -m "Initial authentication schema"
```

### 4.4. Apply Migrations

```bash
alembic upgrade head
```

### 4.5. Verify Tables

Connect to Neon database and verify tables exist:
```sql
\dt  -- List tables (in psql)

-- Should see:
-- users
-- sessions
-- oauth_accounts
-- alembic_version
```

---

## 5. Backend Application Structure

Create the following directory structure:

```
backend/
├── .env
├── .venv/
├── requirements.txt
├── alembic.ini
├── migrations/
│   ├── env.py
│   └── versions/
│       └── 001_initial_auth.py
├── src/
│   ├── __init__.py
│   ├── main.py
│   ├── config.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── session.py
│   │   └── oauth_account.py
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── session.py
│   │   └── auth.py
│   ├── api/
│   │   ├── __init__.py
│   │   ├── deps.py          # Dependencies (get_current_user)
│   │   └── routes/
│   │       ├── __init__.py
│   │       ├── auth.py      # /auth/register, /auth/login, /auth/logout
│   │       ├── oauth.py     # /auth/oauth/google, /auth/oauth/google/callback
│   │       └── session.py   # /auth/session
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth.py          # Password hashing, token generation
│   │   ├── session.py       # Session management
│   │   └── oauth.py         # OAuth provider logic
│   └── db/
│       ├── __init__.py
│       └── session.py       # Database session management
└── tests/
    ├── __init__.py
    ├── conftest.py
    ├── test_auth.py
    ├── test_oauth.py
    └── test_session.py
```

---

## 6. Run Backend Development Server

### 6.1. Start FastAPI Server

```bash
cd backend
source .venv/bin/activate
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 6.2. Verify Server Running

Open browser: [http://localhost:8000/docs](http://localhost:8000/docs)

You should see the FastAPI Swagger UI with authentication endpoints.

---

## 7. Frontend Setup (Docusaurus)

### 7.1. Install Dependencies

```bash
cd frontend  # or wherever Docusaurus is located
npm install
```

### 7.2. Configure Auth Client

Create `frontend/src/utils/auth.ts`:
```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://app.example.com/api'
  : 'http://localhost:8000/api';

export const auth = {
  async register(name: string, email: string, password: string) {
    const response = await fetch(`${API_BASE_URL}/auth/register`, {
      method: 'POST',
      credentials: 'include',  // Send cookies
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, email, password }),
    });
    return response.json();
  },

  async login(email: string, password: string) {
    const response = await fetch(`${API_BASE_URL}/auth/login`, {
      method: 'POST',
      credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
    });
    return response.json();
  },

  async logout() {
    const response = await fetch(`${API_BASE_URL}/auth/logout`, {
      method: 'POST',
      credentials: 'include',
    });
    return response.json();
  },

  async getCurrentSession() {
    const response = await fetch(`${API_BASE_URL}/auth/session`, {
      credentials: 'include',
    });
    return response.json();
  },

  loginWithGoogle() {
    window.location.href = `${API_BASE_URL}/auth/oauth/google`;
  },
};
```

### 7.3. Start Docusaurus Dev Server

```bash
npm start
```

Frontend will be available at: [http://localhost:3000](http://localhost:3000)

---

## 8. Testing the Integration

### 8.1. Test Email/Password Registration

1. Open [http://localhost:8000/docs](http://localhost:8000/docs)
2. Find `POST /api/auth/register`
3. Click "Try it out"
4. Enter:
   ```json
   {
     "name": "Test User",
     "email": "test@example.com",
     "password": "SecurePass123"
   }
   ```
5. Click "Execute"
6. Should receive 201 response with user and session data
7. Cookie should be set in browser

### 8.2. Test Email/Password Login

1. Find `POST /api/auth/login`
2. Enter same email and password
3. Should receive 200 response with session

### 8.3. Test Get Current Session

1. Find `GET /api/auth/session`
2. Execute (cookie should be sent automatically)
3. Should receive user and session data

### 8.4. Test Google OAuth

1. Navigate to: [http://localhost:8000/api/auth/oauth/google](http://localhost:8000/api/auth/oauth/google)
2. Should redirect to Google login
3. Authorize with test user email
4. Should redirect back with session established

### 8.5. Test Logout

1. Find `POST /api/auth/logout`
2. Execute
3. Cookie should be cleared
4. `GET /api/auth/session` should return null values

---

## 9. Development Tools

### 9.1. Database Inspection

**pgAdmin** (GUI):
1. Download from [pgadmin.org](https://www.pgadmin.org/download/)
2. Add server with Neon connection string
3. Browse tables, view data

**psql** (CLI):
```bash
psql "postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/neondb?sslmode=require"
```

### 9.2. API Testing

**Postman**:
1. Import OpenAPI spec from `specs/001-better-auth/contracts/auth-api.yaml`
2. Create requests with cookies enabled

**curl**:
```bash
# Register
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"name":"Test","email":"test@example.com","password":"SecurePass123"}' \
  -c cookies.txt

# Login (save cookies)
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"SecurePass123"}' \
  -c cookies.txt

# Get session (send cookies)
curl http://localhost:8000/api/auth/session -b cookies.txt

# Logout
curl -X POST http://localhost:8000/api/auth/logout -b cookies.txt
```

---

## 10. Common Issues & Troubleshooting

### Issue: Database connection fails

**Solution**:
- Verify `DATABASE_URL` in `.env` is correct
- Check Neon project is not suspended (free tier sleeps after inactivity)
- Ensure `asyncpg` is installed: `pip install asyncpg`

### Issue: OAuth redirect doesn't work

**Solution**:
- Verify redirect URI in Google Cloud Console matches `GOOGLE_REDIRECT_URI` in `.env`
- Check `ALLOWED_ORIGINS` includes frontend URL
- Ensure CORS is configured in FastAPI

### Issue: Cookies not being set

**Solution**:
- Check `SECURE_COOKIES=false` in development (localhost is HTTP)
- Verify `credentials: 'include'` in frontend fetch requests
- Check browser console for CORS errors

### Issue: Rate limiting errors in development

**Solution**:
- Set `RATE_LIMIT_ENABLED=false` in `.env`
- Or install local Redis: `docker run -d -p 6379:6379 redis`

### Issue: Alembic migration fails

**Solution**:
- Drop all tables and run migrations from scratch:
  ```sql
  DROP TABLE oauth_accounts, sessions, users, alembic_version CASCADE;
  ```
- Then: `alembic upgrade head`

---

## 11. Next Steps

After completing the quickstart:

1. **Run tests**: `pytest backend/tests`
2. **Review implementation plan**: Read `specs/001-better-auth/plan.md`
3. **Start implementing tasks**: Follow `specs/001-better-auth/tasks.md` (when created)
4. **Set up CI/CD**: Configure GitHub Actions for automated testing
5. **Deploy to staging**: Set up Vercel/Railway for backend + frontend

---

## 12. Additional Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [SQLAlchemy 2.0 Tutorial](https://docs.sqlalchemy.org/en/20/tutorial/)
- [Alembic Tutorial](https://alembic.sqlalchemy.org/en/latest/tutorial.html)
- [Authlib FastAPI Integration](https://docs.authlib.org/en/latest/client/fastapi.html)
- [Neon Docs](https://neon.tech/docs)
- [Better Auth Documentation](https://better-auth.com) (for pattern reference)

---

**Quickstart Complete**: 2025-12-14
**Ready for**: Implementation phase with `/sp.tasks`
