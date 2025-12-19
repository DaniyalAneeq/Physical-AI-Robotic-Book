# Backend Setup Guide

This guide will help you set up the unified backend for local development.

## Prerequisites

- Python 3.11 or higher
- PostgreSQL database (we recommend Neon Serverless)
- OpenAI API account
- Qdrant Cloud account

## Quick Setup

### 1. Install Dependencies

```bash
cd rag-chatbot/backend
pip install -r requirements.txt
```

### 2. Set Up Environment Variables

```bash
# Copy the template to create your local environment file
cp .env.local.template .env.local

# Edit .env.local and fill in your actual credentials
# This file is gitignored and won't be committed
```

**Required Environment Variables:**

Get these from your service providers:

- **OpenAI API Key**: https://platform.openai.com/api-keys
- **Qdrant**: https://cloud.qdrant.io/
- **Database**: https://neon.tech/

Generate a secure session secret:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 3. Set Up OAuth (Optional)

If you want to enable Google/GitHub OAuth login:

**Google OAuth:**
1. Go to https://console.cloud.google.com/apis/credentials
2. Create OAuth 2.0 Client ID
3. Add authorized redirect URI: `http://localhost:8000/auth/oauth/google/callback`
4. Copy Client ID and Secret to `.env.local`

**GitHub OAuth:**
1. Go to https://github.com/settings/developers
2. Create new OAuth App
3. Set callback URL: `http://localhost:8000/auth/oauth/github/callback`
4. Copy Client ID and Secret to `.env.local`

### 4. Initialize Database

The database tables will be created automatically on first run. Alternatively, you can run migrations manually:

```bash
# Run any pending migrations
alembic upgrade head
```

### 5. Start the Server

```bash
# Development mode with auto-reload
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

The server will start at: http://localhost:8000

- API docs: http://localhost:8000/docs
- Health check: http://localhost:8000/api/health

## Environment File Priority

The backend loads environment variables in this order (later files override earlier ones):

1. `.env.development` - Default development settings (committed to git, no secrets)
2. `.env.local` - Your local overrides (gitignored, contains your actual secrets)
3. System environment variables - Highest priority

**✅ Recommended for development:**
- Keep `.env.development` with placeholder values (safe to commit)
- Create `.env.local` with your actual secrets (gitignored)

**⚠️ Never commit:**
- `.env.local`
- `.env` (deprecated, don't use)
- Any file with actual API keys or secrets

## Troubleshooting

### Import Error: "Could not import unified backend settings"

This warning is expected when running the standalone auth backend. If you see this when running the unified backend:

1. Verify you're in the correct directory: `rag-chatbot/backend/`
2. Check that `app/config.py` exists
3. Ensure all dependencies are installed: `pip install -r requirements.txt`

### Database Connection Error

```
sqlalchemy.exc.OperationalError: could not connect to server
```

**Fix:**
1. Verify your `DATABASE_URL` in `.env.local` is correct
2. Ensure your database allows connections from your IP
3. For Neon: make sure you're using the pooled connection string

### CORS Errors in Browser

```
Access to fetch at 'http://localhost:8000/auth/login' has been blocked by CORS policy
```

**Fix:**
1. Check that your frontend URL is in `CORS_ORIGINS`
2. For localhost, use exact same hostname (don't mix `localhost` and `127.0.0.1`)
3. Verify `allow_credentials=True` is set in CORS middleware

### Cookies Not Being Set

**Symptoms:** Login succeeds but user is logged out on page refresh

**Fix:**
1. For development (HTTP): Ensure `SECURE_COOKIES=false` and `SAME_SITE_COOKIES=lax`
2. For production (HTTPS): Use `SECURE_COOKIES=true` and `SAME_SITE_COOKIES=none`
3. Check browser DevTools → Application → Cookies to verify cookie is set

### OAuth Redirect Error

```
redirect_uri_mismatch
```

**Fix:**
1. Verify `OAUTH_*_REDIRECT_URI` matches exactly what's configured in OAuth provider
2. Include protocol (http:// or https://)
3. Include port if not default (e.g., :8000)
4. No trailing slash

## Production Deployment

See [DEPLOYMENT.md](../../specs/001-fix-auth/DEPLOYMENT.md) for production deployment guide.

Key differences for production:
- Use `.env.production` as template
- Set `SECURE_COOKIES=true`
- Set `SAME_SITE_COOKIES=none`
- Use HTTPS for backend
- Update CORS origins to production frontend URL
- Use production OAuth redirect URIs
- Use strong `SESSION_SECRET` (32+ random characters)

## Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app

# Run specific test file
pytest tests/test_auth.py
```

## Additional Resources

- [Authentication Implementation Summary](../../specs/001-fix-auth/IMPLEMENTATION_SUMMARY.md)
- [Known Issues & Limitations](../../specs/001-fix-auth/known-issues.md)
- [Debugging Guide](../../specs/001-fix-auth/debugging.md)
- [API Documentation](http://localhost:8000/docs) (when server is running)
