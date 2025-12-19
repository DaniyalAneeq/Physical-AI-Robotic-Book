# Feature Specification: Authentication Session Persistence Fix

**Feature Branch**: `001-fix-auth`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Create a detailed specification to fix all authentication issues in the Physical-AI-Robotic-Book project. Context: Frontend: Docusaurus (React, static), Backend: FastAPI (Python), Auth: Better Auth (email/password + Google OAuth), Database: Neon (Postgres), Architecture: API-first, decoupled frontend/backend, Base path: /Physical-AI-Robotic-Book. Problems: 1. Email/password login succeeds, but visiting /profile logs the user out and redirects to /login. 2. Google login shows 'Welcome / login successful' but user is still treated as unauthenticated. 3. Signup successfully creates user in DB, but onboarding popup never appears and user remains unauthenticated."

## Executive Summary

This specification addresses critical authentication session persistence failures in the Physical-AI-Robotic-Book project. Users experience authentication loss after successful login/signup, preventing access to protected routes and causing poor user experience. The root cause is a cross-origin cookie configuration mismatch between the FastAPI backend (port 8000) and Docusaurus frontend (port 3000/GitHub Pages), compounded by missing auth state synchronization.

## Root Cause Analysis

### Problem 1: Email/Password Login Session Loss on Profile Access

**Symptoms**:
- User successfully logs in with email/password
- Visiting `/profile` immediately logs user out and redirects to `/login`

**Root Causes Identified**:

1. **Cookie Domain Mismatch** (`auth.py:94`, `auth.py:174`):
   - Login sets cookie with `domain=None` (browser auto-sets to `localhost`)
   - Cross-origin requests from `localhost:3000` to `localhost:8000` fail cookie transmission
   - Cookie domain should be explicitly set or use credential forwarding

2. **SameSite Policy Misconfiguration** (`config.py:50`):
   - Current: `same_site_cookies = "none"`
   - Requires `secure=True` in production, but `secure_cookies=False` in dev (`config.py:49`)
   - Browser rejects cookies with `SameSite=None` when `Secure=false`

3. **Missing CORS Credentials** (potential):
   - Frontend fetch must include `credentials: "include"` (already implemented in `authApi.ts:102`)
   - Backend CORS must allow credentials (needs verification)

4. **Frontend Auth State Desync** (`useAuth.tsx:50-77`):
   - `loadSession()` called on mount queries backend for session
   - If cookie isn't sent, backend returns 401, clearing frontend auth state
   - No retry or fallback mechanism

### Problem 2: Google OAuth Session Not Persisting

**Symptoms**:
- Google OAuth completes successfully
- User sees "Welcome / login successful" message
- User is immediately treated as unauthenticated
- Onboarding popup never appears

**Root Causes Identified**:

1. **URL Fragment Token Exchange Flow** (`oauth.py:132-140`, `callback.tsx:28-73`):
   - OAuth callback redirects to `/auth/callback` with token in URL fragment
   - Fragment-based token is client-side only (good for security)
   - Frontend calls `/auth/session-from-token` to exchange for cookie
   - If this exchange fails silently, user appears authenticated briefly but loses session

2. **Session Cookie Setting in Exchange** (`auth.py:436-445`):
   - Exchange endpoint sets cookie with same domain/SameSite issues as login
   - Cookie may not persist across redirect to main app

3. **Onboarding State Not Checked After Exchange** (`callback.tsx:68-72`):
   - Code redirects based on `onboarding_required` flag from URL fragment
   - But doesn't verify session was actually established before redirect
   - Results in redirect to onboarding/home without auth state

4. **Frontend Doesn't Wait for Cookie** (`callback.tsx:67-73`):
   - After calling `/session-from-token`, code immediately redirects
   - No verification that subsequent `loadSession()` call will succeed
   - Race condition: redirect happens before cookie is processed by browser

### Problem 3: Signup Success Without Onboarding Popup

**Symptoms**:
- Signup creates user in database successfully
- User receives success response with `onboarding_required: true`
- Onboarding popup never appears
- User remains unauthenticated in UI

**Root Causes Identified**:

1. **Cookie Not Sent After Signup** (`auth.py:91-100`):
   - Same cookie domain/SameSite issues as login
   - Cookie may be set but not accessible to frontend

2. **Frontend State Update Without Cookie Validation** (`useAuth.tsx:115-131`):
   - `register()` sets `user` in state from API response
   - Doesn't verify cookie was actually set and accessible
   - Subsequent requests fail authentication

3. **Onboarding Modal Trigger Logic Missing**:
   - `register()` returns `AuthResponse` with `onboarding_required` flag
   - Caller must check flag and trigger modal
   - No automatic trigger mechanism in auth hook

4. **Auth Context Doesn't Persist Onboarding State**:
   - After signup, if page refreshes, `loadSession()` is called
   - If cookie doesn't work, session is lost
   - Onboarding state is never recovered

## User Scenarios & Testing

### User Story 1 - Email/Password Authentication Flow (Priority: P1)

A user visits the application, registers with email/password, and immediately accesses their profile without re-authentication.

**Why this priority**: Core authentication must work reliably. This is the most common auth flow and blocks all other features.

**Independent Test**: Can be fully tested by registering a new account and immediately navigating to `/profile` without logout. Delivers functional authentication for email/password users.

**Acceptance Scenarios**:

1. **Given** I am on the registration page, **When** I submit valid registration details, **Then** I am logged in immediately with a persistent session cookie
2. **Given** I just registered successfully, **When** I navigate to `/profile` or any protected route, **Then** I remain authenticated and see my profile data
3. **Given** I close my browser after login, **When** I reopen the app within session expiry (30 days), **Then** my session is restored automatically
4. **Given** I am authenticated, **When** I refresh the page, **Then** my authentication state persists and I remain logged in
5. **Given** I log in with email/password, **When** I open DevTools and check cookies, **Then** I see a valid `session_token` cookie with correct domain and SameSite attributes

---

### User Story 2 - Google OAuth Authentication Flow (Priority: P1)

A user initiates Google OAuth login, completes the Google authorization flow, and is authenticated in the application with onboarding presented if needed.

**Why this priority**: OAuth is a primary authentication method and equally critical as email/password. Failure here blocks a major user segment.

**Independent Test**: Can be tested by clicking "Login with Google", completing OAuth consent, and verifying persistent authentication after redirect. Delivers functional OAuth authentication.

**Acceptance Scenarios**:

1. **Given** I am on the login page, **When** I click "Login with Google" and complete OAuth consent, **Then** I am redirected back to the app with an active, persistent session
2. **Given** Google OAuth completed successfully, **When** the callback handler exchanges the token for a session cookie, **Then** the cookie is set with correct cross-origin attributes
3. **Given** I am a new user logging in via Google, **When** OAuth completes, **Then** I see the onboarding modal automatically before accessing the app
4. **Given** I am a returning user logging in via Google, **When** OAuth completes, **Then** I am redirected directly to the home page with my session active
5. **Given** OAuth callback receives a token in the URL fragment, **When** the frontend exchanges it via `/session-from-token`, **Then** the exchange succeeds and sets a valid session cookie before redirecting

---

### User Story 3 - Onboarding Completion After Signup (Priority: P2)

A new user completes registration and is immediately presented with an onboarding modal to personalize their experience.

**Why this priority**: Onboarding improves user retention and personalization but is secondary to basic authentication functionality.

**Independent Test**: Can be tested by registering a new account and verifying the onboarding modal appears immediately. Delivers personalized onboarding experience.

**Acceptance Scenarios**:

1. **Given** I just completed signup, **When** the registration response includes `onboarding_required: true`, **Then** the onboarding modal appears automatically
2. **Given** I am in the onboarding modal, **When** I submit my preferences, **Then** my profile is updated and `onboarding_completed` is set to true
3. **Given** I close the onboarding modal without completing it, **When** I refresh or navigate away, **Then** the modal reappears on next visit until completed
4. **Given** I complete onboarding, **When** I log out and log back in, **Then** the onboarding modal does not appear again

---

### User Story 4 - Cross-Origin Session Persistence (Priority: P1)

The application maintains session cookies correctly across different origins (localhost dev, GitHub Pages prod) and different ports.

**Why this priority**: Essential for development workflow and production deployment. Without this, authentication cannot work in any environment.

**Independent Test**: Can be tested by logging in on `localhost:3000` (frontend) and verifying cookie is accessible when making requests to `localhost:8000` (backend). Delivers cross-origin authentication support.

**Acceptance Scenarios**:

1. **Given** I am developing locally with frontend on port 3000 and backend on port 8000, **When** I log in, **Then** session cookies work across both ports
2. **Given** the app is deployed on GitHub Pages with separate backend, **When** I authenticate, **Then** session cookies work across both domains
3. **Given** I am using HTTPS in production, **When** cookies are set, **Then** they have `Secure=true` and `SameSite=None` attributes
4. **Given** I am developing locally with HTTP, **When** cookies are set, **Then** they have `Secure=false` and `SameSite=Lax` attributes
5. **Given** CORS is configured correctly, **When** frontend makes authenticated requests, **Then** credentials are included and backend accepts them

---

### User Story 5 - Session Recovery After Page Reload (Priority: P2)

A user refreshes the page or returns to the app, and their authentication state is restored automatically.

**Why this priority**: Important for user experience but not blocking core functionality. Session should persist but initial auth must work first.

**Independent Test**: Can be tested by logging in, refreshing the page, and verifying user remains authenticated. Delivers seamless session persistence.

**Acceptance Scenarios**:

1. **Given** I am authenticated, **When** I refresh the page, **Then** `loadSession()` successfully retrieves my session from the backend
2. **Given** my session cookie is valid, **When** the frontend queries `/auth/session`, **Then** my user data and session are returned
3. **Given** my session has expired, **When** `loadSession()` is called, **Then** I am logged out gracefully and redirected to login
4. **Given** I open the app in a new tab, **When** the page loads, **Then** my existing session is recognized and I am authenticated

---

### Edge Cases

- **What happens when session cookie expires during an active session?**
  - User should see a graceful notification and be redirected to login
  - In-flight requests should handle 401 responses appropriately

- **How does the system handle concurrent logins from multiple devices?**
  - Each device gets its own session token
  - Sessions should not interfere with each other
  - User can manage active sessions (future enhancement)

- **What happens if OAuth callback receives invalid or expired token?**
  - User sees error message
  - Redirected to login page with error context
  - No partial auth state left behind

- **How does the system handle browser cookie blocking or privacy modes?**
  - Detect cookie availability on initial load
  - Display informative message if cookies are blocked
  - Graceful degradation (no auth available without cookies)

- **What happens during OAuth flow if user closes browser mid-redirect?**
  - State is cleaned up on next visit
  - No orphaned sessions or incomplete auth states

- **How does the system handle cookie domain mismatches in deployed environments?**
  - Configuration validates cookie settings on startup
  - Logs warnings if CORS/cookie settings appear incorrect
  - Documentation includes deployment checklist

- **What happens if onboarding modal is dismissed without completion?**
  - Flag remains `onboarding_completed: false`
  - Modal reappears on next authenticated visit
  - User can access app but is reminded to complete onboarding

## Requirements

### Functional Requirements

#### Session Management

- **FR-001**: System MUST set session cookies with explicit domain configuration appropriate for the deployment environment (localhost for dev, production domain for prod)
- **FR-002**: System MUST configure session cookies with `SameSite=Lax` for local development (HTTP) and `SameSite=None; Secure` for production (HTTPS)
- **FR-003**: System MUST set session cookie `Max-Age` to match configured session expiry (30 days default)
- **FR-004**: System MUST set session cookies with `HttpOnly=true` flag to prevent JavaScript access
- **FR-005**: System MUST include `Path=/` in session cookies to ensure availability across all routes
- **FR-006**: Frontend MUST include `credentials: "include"` in all fetch requests to backend authenticated endpoints
- **FR-007**: Backend CORS MUST include `Access-Control-Allow-Credentials: true` header in responses
- **FR-008**: Backend CORS MUST explicitly list allowed origins (no wildcard `*` when credentials are enabled)

#### Email/Password Authentication

- **FR-009**: Login endpoint MUST set session cookie immediately upon successful authentication
- **FR-010**: Registration endpoint MUST set session cookie immediately upon successful account creation
- **FR-011**: Login and registration responses MUST include `onboarding_required` boolean flag
- **FR-012**: Logout endpoint MUST clear session cookie with matching domain/path attributes used when setting it
- **FR-013**: Protected routes MUST validate session cookie on every request and return 401 if invalid/missing

#### OAuth Authentication

- **FR-014**: OAuth callback MUST exchange authorization code for user session and return token in URL fragment (not query parameter)
- **FR-015**: Frontend OAuth callback handler MUST extract token from URL fragment and call `/session-from-token` endpoint
- **FR-016**: `/session-from-token` endpoint MUST validate token, create session cookie, and return user data
- **FR-017**: OAuth callback MUST include `onboarding_required` flag in URL fragment
- **FR-018**: Frontend MUST wait for `/session-from-token` response before redirecting to app
- **FR-019**: Frontend MUST verify session establishment by calling `/auth/session` before final redirect
- **FR-020**: OAuth errors MUST redirect to login page with descriptive error message in query parameter

#### Onboarding Flow

- **FR-021**: Registration response MUST set `onboarding_required: true` for new users
- **FR-022**: Frontend MUST automatically trigger onboarding modal when `onboarding_required: true` is detected
- **FR-023**: Onboarding modal MUST persist (reappear) until user completes onboarding or explicitly dismisses
- **FR-024**: Onboarding completion MUST update `onboarding_completed: true` in user record
- **FR-025**: Auth responses MUST check `onboarding_completed` flag and set `onboarding_required` accordingly

#### Frontend Auth State Management

- **FR-026**: Auth context MUST query backend for current session on initial page load
- **FR-027**: Auth context MUST handle 401 responses by clearing user state and redirecting to login
- **FR-028**: Auth context MUST store user data in React state upon successful authentication
- **FR-029**: Auth context MUST provide `refreshSession()` method to manually revalidate session
- **FR-030**: Protected routes MUST use `useRequireAuth()` hook to enforce authentication

#### Session Validation & Recovery

- **FR-031**: Backend MUST validate session token on every authenticated request
- **FR-032**: Backend MUST check session expiry timestamp and reject expired sessions with 401
- **FR-033**: Frontend MUST retry failed requests due to expired session by calling logout flow
- **FR-034**: System MUST log authentication failures (invalid tokens, expired sessions) for monitoring

### Key Entities

- **User**: Represents an authenticated account with email, name, email verification status, onboarding completion status, and preferred locale
- **Session**: Represents an active authentication session with token (hashed in DB), user ID, IP address, user agent, creation time, last used time, and expiry time
- **OAuth Account**: Links user to external OAuth provider (Google, GitHub) with provider ID, provider user ID, access tokens, and refresh tokens
- **Onboarding Profile**: Stores user's onboarding responses including user type, area of interest, experience level, and topics of interest

### Configuration Requirements

- **FR-035**: Backend MUST support environment-specific cookie configuration (`SECURE_COOKIES`, `SAME_SITE_COOKIES`)
- **FR-036**: Backend MUST support environment-specific CORS origins configuration (`CORS_ORIGINS`)
- **FR-037**: Frontend MUST detect environment (localhost vs production) and use correct API base URL
- **FR-038**: Configuration MUST include OAuth redirect URIs matching actual deployment URLs
- **FR-039**: Documentation MUST include deployment checklist for cookie/CORS configuration

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can log in with email/password and immediately access protected routes without re-authentication (100% success rate in testing)
- **SC-002**: Users can authenticate via Google OAuth and remain authenticated after callback redirect (100% success rate in testing)
- **SC-003**: New users see onboarding modal within 2 seconds of completing registration or OAuth signup
- **SC-004**: Session cookies persist across page refreshes for the configured session lifetime (30 days)
- **SC-005**: Authentication works correctly in both development (localhost, HTTP, different ports) and production (HTTPS, single domain) environments
- **SC-006**: Session validation failures (expired, invalid tokens) result in graceful logout and redirect to login (no console errors, no blank screens)
- **SC-007**: Zero authentication-related support tickets or bug reports after deployment of fixes
- **SC-008**: All authentication flows (email/password login, email/password signup, Google OAuth) pass automated E2E tests with 100% success rate
- **SC-009**: DevTools inspection shows session cookies with correct attributes (domain, SameSite, Secure, HttpOnly, Path)
- **SC-010**: Backend returns correct CORS headers (`Access-Control-Allow-Credentials: true`, explicit origins) for all authenticated requests

## Testing Requirements

### Unit Tests

#### Backend (Python/pytest)

1. **Session Service Tests**:
   - Test session creation sets correct cookie attributes based on environment
   - Test session validation rejects expired sessions
   - Test session validation rejects invalid tokens
   - Test session refresh creates new token and revokes old one

2. **Auth Route Tests**:
   - Test `/auth/register` sets session cookie with correct attributes
   - Test `/auth/login` sets session cookie with correct attributes
   - Test `/auth/logout` clears session cookie with matching attributes
   - Test `/auth/session` returns 401 when no cookie present
   - Test `/auth/session` returns user data when valid cookie present

3. **OAuth Route Tests**:
   - Test `/auth/oauth/google/callback` creates session and redirects with token in fragment
   - Test `/auth/session-from-token` validates token and sets session cookie
   - Test OAuth callback includes `onboarding_required` in redirect URL

4. **Configuration Tests**:
   - Test cookie settings are correct for development environment (Lax, not Secure)
   - Test cookie settings are correct for production environment (None, Secure)
   - Test CORS origins are parsed correctly from environment variable

#### Frontend (TypeScript/Jest)

1. **Auth API Tests**:
   - Test all API calls include `credentials: "include"`
   - Test `getCurrentSession()` handles 401 gracefully
   - Test `loginWithGoogle()` navigates to correct OAuth URL

2. **Auth Hook Tests**:
   - Test `loadSession()` updates user state on success
   - Test `loadSession()` clears user state on 401
   - Test `register()` returns `AuthResponse` with `onboarding_required` flag
   - Test `logout()` clears user state

3. **OAuth Callback Tests**:
   - Test token extraction from URL fragment
   - Test `/session-from-token` exchange before redirect
   - Test redirect to onboarding when `onboarding_required: true`
   - Test redirect to home when `onboarding_required: false`

### Integration Tests

1. **Email/Password Login Flow**:
   - Register new user → verify session cookie set → navigate to `/profile` → verify authenticated

2. **Google OAuth Flow**:
   - Initiate OAuth → complete Google consent (mocked) → callback exchange → verify session cookie → verify user authenticated

3. **Onboarding Flow**:
   - Register new user → verify onboarding modal appears → complete onboarding → verify `onboarding_completed: true`

4. **Session Persistence**:
   - Login → store cookie → simulate page refresh → verify session restored from cookie

5. **Cross-Origin Cookie Test**:
   - Frontend (port 3000) makes request to backend (port 8000) → verify cookie sent in request → verify response includes cookie

### E2E Tests (Playwright)

1. **Complete Authentication Journey**:
   ```
   1. Navigate to /login
   2. Click "Register" tab
   3. Fill registration form with valid data
   4. Submit form
   5. Verify onboarding modal appears
   6. Complete onboarding form
   7. Submit onboarding
   8. Verify redirected to home page
   9. Verify user name displayed in navbar
   10. Navigate to /profile
   11. Verify profile page loads (not redirected to /login)
   12. Refresh page
   13. Verify still authenticated (name still in navbar)
   14. Open DevTools → Application → Cookies
   15. Verify session_token cookie exists with correct attributes
   ```

2. **Google OAuth Journey**:
   ```
   1. Navigate to /login
   2. Click "Login with Google"
   3. Complete Google OAuth flow (use Playwright OAuth mocking)
   4. Verify redirected to /auth/callback
   5. Wait for "Welcome" message
   6. Verify redirected to home (or onboarding if new user)
   7. Verify user authenticated (name in navbar)
   8. Verify session cookie exists in DevTools
   ```

3. **Logout and Session Expiry**:
   ```
   1. Login successfully
   2. Verify authenticated
   3. Click "Logout"
   4. Verify redirected to home
   5. Verify user not authenticated (no name in navbar)
   6. Navigate to /profile
   7. Verify redirected to /login
   8. Verify session cookie cleared in DevTools
   ```

4. **Session Expiry Handling**:
   ```
   1. Login successfully
   2. Manually delete session cookie via DevTools
   3. Navigate to /profile
   4. Verify graceful redirect to /login (no errors in console)
   ```

### Regression Tests

1. **Re-login After Logout**: Ensure logout → login → profile access works without issues
2. **Multiple Tab Sessions**: Open app in two tabs, logout in one, verify other tab handles session loss gracefully
3. **OAuth State Tampering**: Attempt to modify OAuth state/token, verify security errors handled correctly
4. **Cookie Blocking**: Test with browser privacy mode that blocks third-party cookies, verify informative error message

## Out of Scope

The following items are explicitly excluded from this specification:

1. **Rewriting the Entire Auth System**: We will not replace FastAPI session-based auth with Better Auth framework. The fix focuses on cookie configuration and state management.

2. **Multi-Factor Authentication (MFA)**: Not included in this fix. Can be added as future enhancement.

3. **Social Login Providers Beyond Google**: GitHub OAuth is present but not part of this bug fix. Google OAuth is prioritized.

4. **Session Refresh/Renewal Logic**: Automatic session extension before expiry is not included. Current 30-day expiry is sufficient.

5. **Advanced Session Management UI**: Viewing active sessions, revoking specific sessions from UI is out of scope.

6. **Email Verification Flow**: Email verification is a database flag but sending verification emails is not part of this fix.

7. **Password Reset Flow**: Not broken, not part of this specification.

8. **Rate Limiting for Auth Endpoints**: Important for production but not related to current bugs.

9. **Backend Migration to Better Auth Framework**: The existing FastAPI implementation works correctly; issues are configuration-related, not architectural.

## Assumptions

1. **FastAPI CORS Middleware Configured**: Assumes FastAPI app has CORS middleware correctly configured to allow credentials
2. **Neon Database Connectivity**: Assumes database is accessible and session table schema is correct
3. **OAuth Credentials Valid**: Assumes Google OAuth client ID/secret are valid and redirect URIs match configuration
4. **HTTPS in Production**: Assumes production deployment uses HTTPS (required for `SameSite=None; Secure`)
5. **Browser Cookie Support**: Assumes users have cookies enabled (standard requirement for session-based auth)
6. **Same Backend for All Environments**: Assumes single backend deployment serves both localhost dev and production frontend

## Dependencies

- **Backend**: FastAPI 0.115+, Starlette CORS middleware, psycopg3
- **Frontend**: React 19.0.0, Docusaurus 3.9.2, fetch API
- **Database**: Neon Serverless Postgres
- **Testing**: pytest (backend), Jest (frontend), Playwright (E2E)

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Cookie SameSite=None requires HTTPS in prod | High | Medium | Document requirement, add config validation, use Lax for dev |
| Browser blocks third-party cookies in privacy mode | Medium | Low | Detect and show informative message, document limitation |
| CORS preflight failures in production | High | Low | Test CORS config in staging before prod, add monitoring |
| Session cookie not sent due to domain mismatch | High | Low | Validate cookie domain matches deployment, add startup checks |
| Race condition in OAuth callback token exchange | Medium | Medium | Add explicit wait/verification before redirect |
| Onboarding modal state lost on refresh | Low | Low | Store onboarding trigger in session, recheck on load |

## Implementation Notes

### Backend Changes Required

1. **Update Cookie Configuration** (`config.py`):
   - Add environment detection (dev vs prod)
   - Set `secure_cookies` and `same_site_cookies` based on environment
   - Validate configuration on startup

2. **Standardize Cookie Setting** (`auth.py`, `oauth.py`):
   - Create helper function for cookie attributes
   - Use consistent domain/path/SameSite/Secure across all endpoints
   - Ensure logout uses same attributes for deletion

3. **CORS Configuration**:
   - Verify `allow_credentials=True` in CORS middleware
   - Ensure origins list is explicit (no wildcard with credentials)
   - Add `Access-Control-Allow-Headers` for all needed headers

4. **OAuth Callback Improvements** (`oauth.py`):
   - Add error logging for token exchange failures
   - Return detailed error responses (sanitized for security)

### Frontend Changes Required

1. **OAuth Callback Handler** (`callback.tsx`):
   - Add session verification step after `/session-from-token`
   - Call `/auth/session` to confirm cookie worked before redirect
   - Add retry logic if session establishment fails
   - Improve error handling and user feedback

2. **Auth Hook** (`useAuth.tsx`):
   - Add onboarding modal trigger logic based on `onboarding_required` flag
   - Improve error handling in `loadSession()`
   - Add session verification method

3. **Registration Flow**:
   - Check `onboarding_required` in response
   - Trigger onboarding modal automatically
   - Store onboarding state to persist across refreshes

### Configuration Changes Required

1. **Development** (`.env` in `auth_backend`):
   ```
   SECURE_COOKIES=false
   SAME_SITE_COOKIES=lax
   CORS_ORIGINS=http://localhost:3000,http://localhost:8000
   FRONTEND_URL=http://localhost:3000/Physical-AI-Robotic-Book
   ```

2. **Production** (environment variables):
   ```
   SECURE_COOKIES=true
   SAME_SITE_COOKIES=none
   CORS_ORIGINS=https://daniyalaneeq.github.io
   FRONTEND_URL=https://daniyalaneeq.github.io/Physical-AI-Robotic-Book
   ```

## Next Steps

1. **Review & Approve Specification**: Stakeholder sign-off on requirements and approach
2. **Run `/sp.clarify`**: If any requirements are ambiguous or need stakeholder input
3. **Run `/sp.plan`**: Generate detailed technical implementation plan
4. **Run `/sp.tasks`**: Create granular, testable task list
5. **Implementation**: Execute tasks following TDD approach (red-green-refactor)
6. **Testing**: Run unit, integration, and E2E tests
7. **Deployment**: Update production configuration, deploy fixes
8. **Validation**: Verify all success criteria met in production environment
