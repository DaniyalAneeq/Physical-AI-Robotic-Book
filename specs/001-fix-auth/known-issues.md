# Known Issues and Limitations

This document outlines known limitations, edge cases, and browser-specific behaviors for the authentication system.

## Session Management

### 1. Session Expiry During Active Use

**Issue**: User sessions expire after 30 days absolute or 7 days of inactivity, even if the user is actively using the application.

**Impact**:
- User will be logged out mid-session if 30-day limit reached
- User will be logged out if they don't use the app for 7 consecutive days

**Workaround**:
- Implement "Keep me logged in" feature (future enhancement)
- Add session expiry warning banner before expiry
- Auto-refresh session on user activity

**Technical Details**:
- Absolute expiration: 30 days from session creation (`session.created_at + 30 days`)
- Idle timeout: 7 days since last use (`session.last_used_at + 7 days`)
- `last_used_at` updates on each authenticated request

### 2. Concurrent Logins from Same Account

**Behavior**: Users can log in from multiple devices/browsers simultaneously.

**Current Implementation**:
- Each login creates a new session
- All sessions remain valid until they expire or are explicitly logged out
- Logging out from one device does NOT log out other devices

**Implications**:
- User may have 5+ active sessions across devices
- No session limit enforced
- Old sessions persist until 30-day expiry

**Future Enhancements**:
- Add "Active Sessions" management page
- Allow users to view and revoke sessions remotely
- Implement max concurrent sessions limit (e.g., 5 devices)
- Add "Log out all devices" option

### 3. Session Recovery After Network Issues

**Issue**: If network fails during critical session operations, state may be inconsistent.

**Scenarios**:

**Login succeeds but cookie not set:**
- Backend creates session and responds with 200 OK
- Network fails before response reaches browser
- Cookie never set, user appears logged out
- Session exists in database but is orphaned

**Logout request fails:**
- User clicks logout
- Request fails due to network
- Cookie remains in browser
- Backend session NOT revoked

**Mitigation**:
- Frontend should retry failed login/logout operations
- Implement logout confirmation
- Backend should clean up orphaned sessions periodically

## Browser and Cookie Behavior

### 4. Third-Party Cookie Blocking

**Issue**: Some browsers and privacy extensions block cookies in cross-origin contexts, even with `SameSite=None; Secure`.

**Affected Scenarios**:
- Safari with "Prevent Cross-Site Tracking" enabled (default)
- Firefox with Enhanced Tracking Protection set to "Strict"
- Chrome with third-party cookies disabled
- Privacy-focused browsers (Brave, Firefox Focus)

**Symptoms**:
- Authentication works in development (same-origin)
- Fails in production (cross-origin)
- No error shown to user, session simply doesn't persist

**Workaround**:
1. **Same-origin deployment** (recommended):
   - Deploy frontend and backend on same domain using subdomain or reverse proxy
   - Example: `app.example.com` (frontend) and `api.example.com` (backend)
   - Or: `example.com` (frontend) and `example.com/api` (backend via proxy)

2. **User instruction**:
   - Add banner: "Authentication requires cookies. Please enable cookies for this site."
   - Provide browser-specific instructions

3. **Fallback to alternative auth**:
   - Use URL-based tokens (less secure, not recommended)
   - Implement local storage + CSRF protection (security trade-offs)

**Technical Details**:
- `SameSite=None` allows cross-site cookies
- Requires `Secure=true` (HTTPS only)
- Still subject to browser tracking protection policies

### 5. Incognito/Private Mode Limitations

**Issue**: Sessions in private browsing don't persist after closing the browser window.

**Behavior**:
- Session cookie is set correctly
- Works during the private session
- Cookie deleted when private window closes
- User must re-authenticate on next private session

**Expected Behavior**: This is by design - private mode intentionally clears all cookies.

**Note**: 30-day cookie expiration doesn't apply in private mode due to browser behavior.

### 6. Cookie Size Limits

**Current Implementation**:
- Session token: ~64 characters (base64-encoded UUID + timestamp + signature)
- Cookie total size: ~200 bytes

**Browser Limits**:
- Per-cookie: 4KB (we use ~0.2KB, safe)
- Per-domain: 50-180 cookies depending on browser

**Risk**: LOW - we only use 1 cookie, well under limits.

**Future Consideration**: If adding more cookie-based features, track total cookie size.

## OAuth-Specific Issues

### 7. OAuth State Parameter Validation

**Current Implementation**: OAuth state is generated but not securely stored.

**Issue**:
- State parameter passed through URL
- No server-side storage of valid states
- Potential CSRF vulnerability if state is predictable

**Mitigation (Implemented)**:
- State generated with cryptographically secure random
- State validated by OAuth provider
- Short-lived state (expires with OAuth flow timeout)

**Future Enhancement**:
- Store state in Redis/database with expiration
- Validate state matches before processing callback

### 8. OAuth Token Expiry

**Issue**: OAuth access tokens from Google/GitHub expire (typically 1 hour).

**Current Implementation**:
- Access token stored in `oauth_accounts` table
- No automatic refresh
- Token used only during OAuth flow, not afterwards

**Impact**: LOW - Token only needed during initial auth, not for ongoing session.

**Note**: If future features need ongoing API access (e.g., sync Google contacts), implement refresh token logic.

### 9. OAuth Email Verification

**Behavior**: OAuth providers (Google, GitHub) provide verified emails by default.

**Current Implementation**:
- Assumes OAuth emails are verified
- Sets `email_verified_at` automatically for OAuth users
- Email/password users have `email_verified_at = null`

**Limitation**: No email verification flow for email/password signup.

**Future Enhancement**: Add email verification for non-OAuth users.

### 10. OAuth Account Linking Edge Cases

**Scenario 1: User registers with email/password, then tries OAuth with same email**
- ✅ Works: Automatically links OAuth account to existing user

**Scenario 2: User signs up with Google, then tries GitHub with same email**
- ✅ Works: Links GitHub to existing Google-created account

**Scenario 3: User signs up with Google (email1), later changes Google email to email2**
- ❌ Limitation: System doesn't update email, still uses email1
- Workaround: User must manually update email in profile

**Scenario 4: User has multiple Google accounts (personal, work)**
- ❌ Limitation: System doesn't distinguish which Google account was used
- Impact: User may accidentally sign in with wrong Google account
- Mitigation: Show current logged-in email on profile page

## Security Considerations

### 11. Session Fixation

**Protection**: NEW session created on each login, old sessions NOT reused.

**Implementation**:
- `POST /auth/login` creates new session, generates new token
- Old session (if exists) remains valid (see concurrent logins)

**Future Enhancement**: Revoke old sessions on new login (configurable).

### 12. CSRF Protection

**Current Implementation**:
- HttpOnly cookies prevent JavaScript access
- SameSite cookie attribute provides CSRF protection:
  - Development: `SameSite=Lax` (blocks cross-site POST)
  - Production: `SameSite=None` (allows cross-origin, requires explicit consent)

**Limitation**: `SameSite=None` in production reduces CSRF protection.

**Mitigation**:
- CORS restricts origins to known frontends
- Cookies require explicit `credentials: 'include'` in fetch requests
- OAuth flow includes state parameter for additional CSRF protection

**Future Enhancement**: Add CSRF token for state-changing operations.

### 13. Password Reset Flow

**Current Implementation**: Not implemented.

**Workaround**: Users must contact support to reset password.

**Future Enhancement**:
- Email-based password reset
- Security questions
- Multi-factor authentication recovery

## Performance Considerations

### 14. Session Validation Latency

**Current Implementation**:
- Every authenticated request validates session (database query)
- Typical latency: 10-50ms (Neon Serverless Postgres)

**Optimization Opportunities**:
- Cache session validation results (Redis)
- Use short-lived JWT with refresh tokens
- Batch session updates (e.g., update `last_used_at` once per minute, not every request)

**Current Performance**: Acceptable for typical workloads (<1000 req/sec).

### 15. Database Session Cleanup

**Current Implementation**:
- Expired sessions remain in database
- Cleanup function exists: `SessionService.cleanup_expired_sessions()`
- Not automatically run

**Impact**:
- Database grows with old sessions
- No performance impact (queries filter by `expires_at`)
- Storage cost increases over time

**Recommendation**: Run cleanup as daily cron job:
```python
# Cron job (daily at 3 AM)
from auth_backend.services.session import SessionService

session_service = SessionService(secret_key)
count = await session_service.cleanup_expired_sessions(db)
print(f"Cleaned up {count} expired sessions")
```

### 16. OAuth Callback Performance

**Current Implementation**:
- OAuth callback creates user + session in one transaction
- Account linking checks for existing email
- Typical latency: 100-300ms

**Bottleneck**: Database queries for user lookup and creation.

**Acceptable**: OAuth is infrequent operation, latency is reasonable.

## Deployment-Specific Issues

### 17. Environment Variable Mismatch

**Issue**: Using development config in production (or vice versa) breaks authentication.

**Common Mistakes**:
- Forgot to set `SECURE_COOKIES=true` in production
- Used `SameSite=lax` in production (should be `none`)
- `CORS_ORIGINS` includes `localhost` in production

**Symptoms**:
- Cookies not set in production
- "Cross-site cookie" errors
- CORS errors

**Prevention**:
- Use separate `.env.development` and `.env.production` files
- Deployment checklist (see DEPLOYMENT.md)
- Environment validation on startup

### 18. SESSION_SECRET Rotation

**Issue**: Changing `SESSION_SECRET` invalidates ALL existing sessions.

**Impact**:
- All users logged out immediately
- Sessions cannot be recovered

**Best Practices**:
- Do NOT rotate SECRET in production without warning
- If rotation needed:
  1. Announce maintenance window
  2. Rotate secret
  3. Expect all users to re-authenticate
- Consider multi-key rotation (support old + new key for grace period)

### 19. HTTPS Mixed Content

**Issue**: Frontend on HTTPS calling backend on HTTP.

**Browser Behavior**: Blocked by mixed content policy.

**Solution**: Backend MUST use HTTPS in production.

**Common Scenario**: Using Render/Railway free tier (provides HTTPS) but forgot to use `https://` URL.

## Browser-Specific Quirks

### 20. Safari Cookie Behavior

**Issue**: Safari requires user interaction before setting cookies in some contexts.

**Impact**: Minimal - login requires user click/submit.

**Note**: This doesn't affect our implementation.

### 21. iOS Safari Private Browsing

**Issue**: iOS Safari in private mode blocks ALL third-party cookies, regardless of settings.

**Workaround**: Same-origin deployment (see issue #4).

### 22. Chrome "Cookies without SameSite" Warnings

**Behavior**: Chrome DevTools may show warnings even when working correctly.

**Example Warning**: "Cookie was not sent because SameSite=None requires Secure flag"

**Resolution**: Ensure both conditions met:
- `SameSite=None`
- `Secure=true`
- Backend served over HTTPS

## Future Enhancements

### Planned Features

1. **Remember Me** - Extended session duration (90 days)
2. **Session Management UI** - View/revoke active sessions
3. **Email Verification** - Verify email for password-based signups
4. **Password Reset** - Self-service password recovery
5. **MFA/2FA** - Two-factor authentication
6. **Social Providers** - Add Microsoft, Apple, LinkedIn OAuth
7. **Session Activity Log** - Track login location, device, time
8. **Account Deletion** - GDPR-compliant account deletion
9. **Same-Origin Deployment Guide** - Reverse proxy setup for better cookie support

### Performance Improvements

1. **Redis Session Cache** - Reduce database load
2. **JWT Hybrid** - Short-lived JWT + refresh tokens
3. **Connection Pooling** - Optimize database connections
4. **CDN Integration** - Faster OAuth redirect handling

## Support and Debugging

For troubleshooting authentication issues, see:
- [debugging.md](debugging.md) - Comprehensive debugging guide
- [DEPLOYMENT.md](DEPLOYMENT.md) - Environment configuration
- [README.md](../../rag-chatbot/auth_backend/README.md) - API documentation

For new issues not covered here, check:
1. Browser DevTools → Application → Cookies
2. Browser DevTools → Network → Headers
3. Backend server logs
4. Database session records
