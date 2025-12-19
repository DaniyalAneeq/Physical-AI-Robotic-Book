# Deployment Guide: Authentication Session Persistence Fix

This guide explains how to deploy the authentication fix to development and production environments.

## Environment Configuration

### Development Environment (HTTP localhost)

**Cookie Configuration:**
- `SECURE_COOKIES=false` - Cookies work over HTTP
- `SAME_SITE_COOKIES=lax` - Allows same-site requests (localhost:3000 → localhost:8000)

**CORS Configuration:**
- `CORS_ORIGINS=http://localhost:3000,http://localhost:8000`

**Setup Steps:**

1. Copy environment template:
   ```bash
   cd rag-chatbot/backend
   cp .env.development .env
   ```

2. Update the following values in `.env`:
   - `DATABASE_URL` - Your local PostgreSQL connection string
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `QDRANT_URL` and `QDRANT_API_KEY` - Your Qdrant instance details
   - OAuth credentials (if testing OAuth)

3. Start the backend:
   ```bash
   uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
   ```

4. Start the frontend (in separate terminal):
   ```bash
   cd AIdd-book
   npm start
   ```

5. Test authentication:
   - Navigate to `http://localhost:3000/Physical-AI-Robotic-Book`
   - Register a new account
   - Verify session cookie in DevTools (Application → Cookies)
   - Cookie should have: `SameSite=Lax`, `Secure=false`, `HttpOnly=true`

### Production Environment (HTTPS)

**Cookie Configuration:**
- `SECURE_COOKIES=true` - Cookies require HTTPS
- `SAME_SITE_COOKIES=none` - Allows cross-origin requests (GitHub Pages ↔ Backend)

**CORS Configuration:**
- `CORS_ORIGINS=https://yourusername.github.io,https://your-backend-domain.com`

**CRITICAL REQUIREMENTS:**
- ✅ Backend MUST be served over HTTPS (not HTTP)
- ✅ Frontend MUST be served over HTTPS (GitHub Pages provides this)
- ✅ `SameSite=None` REQUIRES `Secure=true` (browsers will reject otherwise)

**Setup Steps:**

1. Set environment variables in your deployment platform (Render, Railway, etc.):

   ```bash
   # Required variables
   ENVIRONMENT=production
   DATABASE_URL=<your-production-db-url>
   OPENAI_API_KEY=<your-openai-key>
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-qdrant-key>
   SESSION_SECRET=<generate-secure-random-secret>

   # Cookie configuration (CRITICAL)
   SECURE_COOKIES=true
   SAME_SITE_COOKIES=none

   # CORS configuration
   CORS_ORIGINS=https://yourusername.github.io,https://your-backend-domain.com

   # Frontend URL
   FRONTEND_URL=https://yourusername.github.io/Physical-AI-Robotic-Book

   # OAuth (if using)
   OAUTH_GOOGLE_CLIENT_ID=<your-production-client-id>
   OAUTH_GOOGLE_CLIENT_SECRET=<your-production-client-secret>
   OAUTH_GOOGLE_REDIRECT_URI=https://your-backend-domain.com/auth/oauth/google/callback
   ```

2. Deploy backend to your hosting platform

3. Update frontend environment variables (if needed):
   ```bash
   # In AIdd-book/.env.production or build config
   REACT_APP_API_URL=https://your-backend-domain.com
   ```

4. Deploy frontend to GitHub Pages

5. Test authentication:
   - Navigate to your production URL
   - Register a new account
   - Verify session cookie in DevTools
   - Cookie should have: `SameSite=None`, `Secure=true`, `HttpOnly=true`

## Troubleshooting

### Issue: Cookies not being set in development

**Symptoms:**
- User logs in successfully but session is not persisted
- Cookie not visible in DevTools

**Solution:**
1. Verify `.env` has:
   ```
   SECURE_COOKIES=false
   SAME_SITE_COOKIES=lax
   ```
2. Ensure frontend and backend are both on `localhost` (not mixing localhost/127.0.0.1)
3. Clear browser cookies and try again

### Issue: Cookies not working in production

**Symptoms:**
- Authentication works in dev but not in production
- "Cross-site cookie" warnings in browser console

**Possible Causes & Solutions:**

1. **Backend not using HTTPS:**
   - `SameSite=None` requires `Secure=true` which requires HTTPS
   - Solution: Ensure backend is deployed with HTTPS (most platforms provide this)

2. **Missing CORS configuration:**
   - Browser blocking cross-origin cookies
   - Solution: Verify `CORS_ORIGINS` includes your frontend URL
   - Verify `allow_credentials=True` in CORS middleware

3. **Incorrect cookie attributes:**
   - Production must use `SECURE_COOKIES=true` and `SAME_SITE_COOKIES=none`
   - Check environment variables are set correctly

4. **Browser blocking third-party cookies:**
   - Some browsers/extensions block all third-party cookies
   - Solution: Test in incognito mode, check browser cookie settings

### Issue: OAuth redirect fails

**Symptoms:**
- OAuth login redirects but session not established

**Solution:**
1. Verify `OAUTH_GOOGLE_REDIRECT_URI` matches the URI configured in Google Cloud Console
2. Check frontend `/auth/callback` page is properly handling token exchange
3. Verify `session-from-token` endpoint is setting cookies correctly

## Security Checklist

Before deploying to production:

- [ ] `SESSION_SECRET` is a strong random value (not default)
- [ ] `SECURE_COOKIES=true` in production
- [ ] `SAME_SITE_COOKIES=none` in production
- [ ] Backend is served over HTTPS
- [ ] `CORS_ORIGINS` contains only trusted domains (no wildcards)
- [ ] OAuth secrets are environment variables (not hardcoded)
- [ ] Database credentials are secure
- [ ] API keys are not exposed in frontend code

## Environment Variable Reference

| Variable | Development | Production | Description |
|----------|-------------|------------|-------------|
| `SECURE_COOKIES` | `false` | `true` | Enable secure flag (requires HTTPS) |
| `SAME_SITE_COOKIES` | `lax` | `none` | SameSite attribute for cookies |
| `CORS_ORIGINS` | `http://localhost:3000,...` | `https://yourusername.github.io,...` | Allowed CORS origins |
| `ENVIRONMENT` | `development` | `production` | Environment identifier |
| `DATABASE_URL` | Local DB | Production DB | PostgreSQL connection string |
| `SESSION_SECRET` | Dev secret | Strong random | Session encryption key |
| `FRONTEND_URL` | `http://localhost:3000/...` | `https://yourusername.github.io/...` | Frontend base URL |

## Testing Authentication

### Manual Testing Checklist

**Development:**
- [ ] Register new account → session persists across page refresh
- [ ] Login with existing account → session persists
- [ ] Logout → session cleared
- [ ] OAuth login (if configured) → session persists after redirect

**Production:**
- [ ] Register new account → session persists across page refresh
- [ ] Login with existing account → session persists
- [ ] Logout → session cleared
- [ ] OAuth login → session persists after redirect
- [ ] Open app in new tab → session recognized
- [ ] Close browser, reopen → session persists (if within 30 days)

### DevTools Verification

Check cookie attributes in browser DevTools (Application → Cookies):

**Development Cookie:**
```
Name: session_token
Value: <hashed-token>
Domain: localhost
Path: /
Expires: <30 days from now>
HttpOnly: ✓
Secure: ✗
SameSite: Lax
```

**Production Cookie:**
```
Name: session_token
Value: <hashed-token>
Domain: <your-backend-domain>
Path: /
Expires: <30 days from now>
HttpOnly: ✓
Secure: ✓
SameSite: None
```

## Support

If you encounter issues not covered in this guide:
1. Check browser console for error messages
2. Check backend logs for authentication errors
3. Verify all environment variables are set correctly
4. Test in different browser/incognito mode to rule out extension interference
