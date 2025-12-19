# Authentication Debugging Guide

This guide helps diagnose and fix common authentication issues with cookie-based session persistence.

## Quick Diagnosis

### 1. Check Cookie in Browser DevTools

**Chrome/Edge:**
1. Open DevTools (F12)
2. Go to Application tab
3. Expand Cookies in left sidebar
4. Select your domain
5. Look for `session_token` cookie

**Expected Cookie Attributes:**

**Development (HTTP):**
- Name: `session_token`
- Value: Random token string (starts with `session_`)
- Domain: `localhost`
- Path: `/`
- Expires: 30 days from creation
- HttpOnly: ✅ (checked)
- Secure: ❌ (unchecked)
- SameSite: `Lax`

**Production (HTTPS):**
- Name: `session_token`
- Value: Random token string (starts with `session_`)
- Domain: Your backend domain
- Path: `/`
- Expires: 30 days from creation
- HttpOnly: ✅ (checked)
- Secure: ✅ (checked)
- SameSite: `None`

### 2. Check Network Requests

**In DevTools Network tab:**

1. **Login/Register Request:**
   - URL: `http://localhost:8000/auth/login` or `/auth/register`
   - Method: `POST`
   - Status: `200 OK`
   - Response Headers should include: `Set-Cookie: session_token=...`
   - Request Headers should include: `credentials: include`

2. **Session Validation Request:**
   - URL: `http://localhost:8000/auth/session`
   - Method: `GET`
   - Status: `200 OK`
   - Request Headers should include: `Cookie: session_token=...`

3. **CORS Preflight (Production only):**
   - URL: Any backend endpoint
   - Method: `OPTIONS`
   - Response Headers should include:
     - `Access-Control-Allow-Origin: https://your-frontend.com`
     - `Access-Control-Allow-Credentials: true`

### 3. Check Console for Errors

Common error messages and what they mean:

**"Cross-site cookie... was not sent in cross-site request"**
- Cause: SameSite=None requires Secure=true (HTTPS)
- Fix: Ensure backend is HTTPS in production, or use SameSite=Lax for development

**"CORS policy: No 'Access-Control-Allow-Origin' header"**
- Cause: CORS not configured or frontend URL not in allowed origins
- Fix: Add frontend URL to CORS_ORIGINS environment variable

**"Failed to fetch"**
- Cause: Backend not running or wrong URL
- Fix: Verify backend is running, check API_BASE_URL in frontend

## Common Issues

### Issue 1: Cookies Not Being Set in Development

**Symptoms:**
- User logs in successfully (200 OK response)
- No `session_token` cookie appears in DevTools
- Session does not persist after page refresh

**Diagnosis Steps:**
1. Check if `Set-Cookie` header is present in login response
2. Verify frontend and backend are both using `localhost` (not mixing with `127.0.0.1`)
3. Check browser cookie settings (not blocking first-party cookies)

**Solutions:**

**A. Backend not sending Set-Cookie:**
```bash
# Check .env file
SECURE_COOKIES=false
SAME_SITE_COOKIES=lax
```

**B. Domain mismatch:**
```bash
# Both must use localhost:
Frontend: http://localhost:3000
Backend:  http://localhost:8000

# NOT:
Frontend: http://127.0.0.1:3000  # ❌ Wrong!
Backend:  http://localhost:8000
```

**C. Browser blocking cookies:**
- Check browser settings → Privacy → Allow cookies
- Try incognito/private mode
- Disable cookie-blocking extensions

### Issue 2: Cookies Not Working in Production

**Symptoms:**
- Authentication works in development
- Production login succeeds but session doesn't persist
- Browser console shows "cookie blocked" warnings

**Diagnosis Steps:**
1. Verify backend is HTTPS (not HTTP)
2. Check cookie attributes in DevTools
3. Look for CORS errors in console
4. Verify environment variables

**Solutions:**

**A. Backend not using HTTPS:**
```bash
# CRITICAL: SameSite=None REQUIRES Secure=true, which REQUIRES HTTPS
# Solution: Deploy backend to platform with HTTPS (Render, Railway, etc.)
```

**B. Wrong cookie attributes:**
```bash
# Production environment variables:
SECURE_COOKIES=true        # Required for HTTPS
SAME_SITE_COOKIES=none     # Required for cross-origin
```

**C. CORS not allowing credentials:**
```python
# In backend main.py, verify:
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,  # Explicit origins, NO wildcards
    allow_credentials=True,  # REQUIRED for cookies
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**D. Frontend not sending credentials:**
```typescript
// All fetch calls must include:
fetch(url, {
    credentials: 'include',  // REQUIRED
    // ...
});
```

### Issue 3: OAuth Redirect Fails

**Symptoms:**
- OAuth flow redirects back to app
- Session not established after redirect
- Token appears in URL fragment but cookie not set

**Diagnosis Steps:**
1. Check if `/auth/callback` page loads
2. Verify `/session-from-token` endpoint is called
3. Check for errors in `/session-from-token` response

**Solutions:**

**A. Callback page not calling session-from-token:**
```tsx
// AIdd-book/src/pages/auth/callback.tsx should:
const response = await fetch(`${API_BASE_URL}/auth/session-from-token`, {
    method: 'POST',
    body: JSON.stringify({ token: sessionToken }),
    credentials: 'include',  // REQUIRED
});
```

**B. Token exchange failing:**
- Check backend logs for validation errors
- Verify session token hasn't expired
- Ensure token is being read correctly from URL fragment

### Issue 4: Session Expires Immediately

**Symptoms:**
- User can log in
- Session clears immediately or after a few minutes
- Cookie disappears from DevTools

**Diagnosis Steps:**
1. Check cookie expiration time in DevTools
2. Look for session validation errors in backend logs
3. Verify session settings

**Solutions:**

**A. Cookie max-age too short:**
```python
# In backend, verify cookie max_age:
response.set_cookie(
    key=settings.session_cookie_name,
    value=token,
    max_age=settings.session_max_age_days * 24 * 60 * 60,  # Should be 30 days
    # ...
)
```

**B. Session validation rejecting valid tokens:**
- Check backend logs for "invalid session" errors
- Verify `SESSION_SECRET` hasn't changed (would invalidate all tokens)
- Check database session expiry times

### Issue 5: User Logged Out Unexpectedly

**Symptoms:**
- User is logged in
- After some time or action, suddenly logged out
- No explicit logout action

**Possible Causes:**

**A. Session expired (30-day absolute or 7-day idle):**
```python
# Session model checks:
- Absolute expiration: 30 days from creation
- Idle timeout: 7 days since last use
```

**B. Token mismatch (SECRET_KEY changed):**
- If `SESSION_SECRET` changes, all existing sessions become invalid
- Solution: Keep SESSION_SECRET consistent across deployments

**C. Frontend error clearing session:**
- Check browser console for errors
- Verify error handling doesn't unnecessarily clear user state

## Advanced Debugging

### Enable Verbose Logging

**Backend (auth routes):**
```python
import logging
logger = logging.getLogger(__name__)

# In login endpoint:
logger.info(f"Login attempt for {email}")
logger.debug(f"Session created: {session.id}")
```

**Frontend (auth hook):**
```typescript
// In useAuth.tsx:
console.log('[Auth] Login attempt:', email);
console.log('[Auth] Session loaded:', user);
```

### Inspect Session in Database

```sql
-- View all active sessions:
SELECT
    id,
    user_id,
    created_at,
    expires_at,
    last_used_at,
    revoked
FROM sessions
WHERE expires_at > NOW() AND revoked = false
ORDER BY created_at DESC;

-- View specific user's sessions:
SELECT * FROM sessions
WHERE user_id = '<user-uuid>'
ORDER BY created_at DESC;
```

### Test with cURL

**Login and capture cookie:**
```bash
curl -v -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password"}' \
  -c cookies.txt

# Check cookies.txt for session_token
cat cookies.txt
```

**Use cookie for authenticated request:**
```bash
curl -v http://localhost:8000/auth/session \
  -b cookies.txt
```

## Environment Checklist

### Development

- [ ] `.env` file exists in `rag-chatbot/backend/`
- [ ] `SECURE_COOKIES=false`
- [ ] `SAME_SITE_COOKIES=lax`
- [ ] `CORS_ORIGINS` includes `http://localhost:3000`
- [ ] Backend running on `http://localhost:8000`
- [ ] Frontend running on `http://localhost:3000`
- [ ] Both using `localhost` (not `127.0.0.1`)

### Production

- [ ] Environment variables set in deployment platform
- [ ] `SECURE_COOKIES=true`
- [ ] `SAME_SITE_COOKIES=none`
- [ ] `CORS_ORIGINS` includes production frontend URL
- [ ] Backend served over HTTPS
- [ ] Frontend served over HTTPS
- [ ] `SESSION_SECRET` is strong random value
- [ ] OAuth redirect URIs match exactly

## Getting Help

If issues persist after trying these solutions:

1. **Check logs:**
   - Backend server logs
   - Browser console
   - Network tab in DevTools

2. **Document the issue:**
   - Exact steps to reproduce
   - Expected vs actual behavior
   - Screenshots of DevTools (cookie, network, console)
   - Environment (dev vs prod)

3. **Common gotchas:**
   - Mixing localhost and 127.0.0.1
   - Using HTTP in production
   - Forgetting `credentials: 'include'`
   - Wildcards in CORS_ORIGINS
   - SESSION_SECRET changed between deploys
