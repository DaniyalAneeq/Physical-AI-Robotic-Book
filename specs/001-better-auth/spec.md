# Feature Specification: Better Auth Integration

**Feature Branch**: `001-better-auth`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "Create a specification for adding authentication to my existing project using Better Auth and Neon database."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Email/Password Account Creation (Priority: P1)

A new user visits the application and wants to create an account using their name, email and password to access protected features like the RAG chatbot with personalized history.

**Why this priority**: Core authentication foundation. Without user accounts, no other auth features (OAuth, sessions, protected resources) can function. This is the minimum viable authentication system.

**Independent Test**: Can be fully tested by submitting a signup form with name, email, and password, then verifying the user can immediately log in with those credentials and access their authenticated session.

**Acceptance Scenarios**:

1. **Given** a user is on the signup page, **When** they enter a valid name, email, and password, **Then** an account is created, a session is established, and they are redirected to the authenticated application
2. **Given** a user tries to signup with an already registered email, **When** they submit the form, **Then** they receive a clear error message indicating the email is already in use
3. **Given** a user enters a weak password (less than 8 characters), **When** they submit the signup form, **Then** they receive validation feedback requiring a stronger password
4. **Given** a user enters an invalid email format, **When** they submit the signup form, **Then** they receive validation feedback for proper email format

---

### User Story 2 - Email/Password Login (Priority: P1)

An existing user wants to log back into their account using their email and password to access their personalized data and session history.

**Why this priority**: Essential for returning users. Without login, account creation (P1) is useless. These two stories together form the minimal viable authentication system.

**Independent Test**: Create a test user account, log out, then verify the user can successfully log back in with their email and password and access their authenticated session.

**Acceptance Scenarios**:

1. **Given** a registered user is on the login page, **When** they enter correct email and password, **Then** they are authenticated and redirected to the application with an active session
2. **Given** a user enters an incorrect password, **When** they submit the login form, **Then** they receive a generic error message (for security) and remain on the login page
3. **Given** a user enters an unregistered email, **When** they submit the login form, **Then** they receive a generic error message and remain on the login page
4. **Given** a user has attempted to login incorrectly 5 times in 10 minutes, **When** they try again, **Then** they are temporarily blocked with a rate limit message

---

### User Story 3 - Google OAuth Signup (Priority: P2)

A new user wants to quickly create an account using their existing Google account without manually entering credentials, reducing friction in the signup process.

**Why this priority**: Significantly improves user experience and conversion rates but depends on the foundational auth system (P1). Can be added after email/password auth is working.

**Independent Test**: Click "Sign up with Google" button, complete Google OAuth flow, and verify a new user account is created and the user is logged in with an active session.

**Acceptance Scenarios**:

1. **Given** a user clicks "Sign up with Google", **When** they authorize the application via Google OAuth, **Then** a new user account is created with their Google profile data and they are logged in
2. **Given** a user's Google email already exists in the system (from email/password signup), **When** they complete Google OAuth, **Then** the Google account is linked to their existing user record and they are logged in
3. **Given** a user cancels the Google OAuth flow, **When** they are redirected back to the application, **Then** they remain on the signup page with no account created
4. **Given** the OAuth state token is invalid or expired, **When** the callback is processed, **Then** the authentication fails with a security error

---

### User Story 4 - Google OAuth Login (Priority: P2)

An existing user who previously signed up with Google wants to log in using their Google account for a seamless authentication experience.

**Why this priority**: Complements Google OAuth signup (P2). Together they provide a complete OAuth authentication flow. Depends on P1 session management.

**Independent Test**: For a user who previously signed up with Google, click "Login with Google", complete OAuth flow, and verify they are logged in with their existing user account and session.

**Acceptance Scenarios**:

1. **Given** a user who previously signed up with Google clicks "Login with Google", **When** they authorize via Google, **Then** they are logged into their existing account with an active session
2. **Given** a user's Google account is not registered, **When** they complete the Google OAuth login flow, **Then** a new account is created (same as signup flow) and they are logged in
3. **Given** an OAuth error occurs at Google's end, **When** the user is redirected back, **Then** they receive a clear error message and can retry or use alternative login method

---

### User Story 5 - Session Management and Logout (Priority: P1)

An authenticated user wants to securely end their session when finished using the application, especially on shared or public devices.

**Why this priority**: Critical security feature. Without proper logout and session management, P1 auth features are incomplete and insecure. Required for production readiness.

**Independent Test**: Log in as a user, verify session persists across page refreshes, then logout and verify the session is terminated and protected resources are no longer accessible.

**Acceptance Scenarios**:

1. **Given** an authenticated user clicks logout, **When** the logout request completes, **Then** their session is terminated, session cookie is cleared, and they are redirected to the login page
2. **Given** an authenticated user closes their browser, **When** they return within the session expiration period (30 days), **Then** they are still logged in with their existing session
3. **Given** an authenticated user's session has expired, **When** they try to access protected resources, **Then** they are redirected to login with a message indicating session expiration
4. **Given** a user logs in from a new device/browser, **When** the session is created, **Then** it is independent of sessions on other devices (multi-device support)

---

### User Story 6 - Protected Resource Access (Priority: P1)

Authenticated users need to access protected API endpoints (like personalized chatbot history) while unauthenticated users are denied access with clear guidance.

**Why this priority**: The purpose of authentication is to protect resources. Without this, authentication serves no functional purpose. Must be part of the MVP.

**Independent Test**: With an active session, call a protected API endpoint and verify it succeeds. Without a session, verify the same endpoint returns 401 Unauthorized.

**Acceptance Scenarios**:

1. **Given** an authenticated user makes a request to a protected endpoint, **When** their session is valid, **Then** the request succeeds and returns the requested data
2. **Given** an unauthenticated user makes a request to a protected endpoint, **When** no valid session exists, **Then** the request returns 401 Unauthorized with a clear error message
3. **Given** an authenticated user's session expires during use, **When** they make a request to a protected endpoint, **Then** they receive a 401 response and are prompted to re-authenticate
4. **Given** a session token is tampered with, **When** a request is made to a protected endpoint, **Then** the request is rejected with a security error

---

### Edge Cases

- What happens when a user tries to signup with Google using an email that already exists via email/password signup?
  - The Google OAuth account should be linked to the existing user record, creating a unified identity
- How does the system handle concurrent login attempts from the same account on different devices?
  - Multiple active sessions are allowed (multi-device support), each tracked independently
- What happens when the session cookie is deleted manually by the user?
  - The user is treated as unauthenticated and must log in again to access protected resources
- What happens when a user changes their password?
  - All existing sessions should be revoked for security, requiring re-authentication on all devices
- How does the system handle OAuth provider outages (Google is unavailable)?
  - Users see a clear error message and can use email/password login as a fallback (if they have credentials)
- What happens when rate limiting blocks a legitimate user?
  - User receives a clear message with wait time and can contact support if needed
- How does the system handle session token theft or compromise?
  - Token hashing at rest prevents direct database compromise; HTTPS, HttpOnly, Secure cookies prevent client-side theft; session revocation provides manual remediation
- What happens when database connection fails during authentication?
  - Authentication requests fail gracefully with 503 Service Unavailable and retry guidance

## Requirements *(mandatory)*

### Functional Requirements

**Authentication Methods**:

- **FR-001**: System MUST support user signup via email and password, collecting name, email, and password fields
- **FR-002**: System MUST support user login via email and password
- **FR-003**: System MUST support user signup via Google OAuth 2.0
- **FR-004**: System MUST support user login via Google OAuth 2.0
- **FR-005**: System MUST create a unified user identity when a user signs up with Google using an email that already exists from email/password signup

**Password Security**:

- **FR-006**: System MUST hash passwords using a cryptographically secure algorithm (bcrypt or argon2) before storage
- **FR-007**: System MUST enforce minimum password requirements: at least 8 characters, combination of letters and numbers
- **FR-008**: System MUST never store plain-text passwords or transmit them in logs

**Session Management**:

- **FR-009**: System MUST create a secure session upon successful authentication (email/password or OAuth)
- **FR-010**: System MUST store session data in Neon Serverless Postgres with cryptographic token hashing
- **FR-011**: System MUST set session cookies with HttpOnly, Secure (in production), and SameSite attributes
- **FR-012**: System MUST implement session expiration with a configurable TTL (default 30 days)
- **FR-013**: System MUST support sliding session expiration (refresh on activity)
- **FR-014**: System MUST allow users to logout, which immediately revokes their session and clears cookies
- **FR-015**: System MUST support multiple concurrent sessions per user (multi-device login)
- **FR-016**: System MUST provide an endpoint to retrieve the current authenticated user and session status

**OAuth Integration**:

- **FR-017**: System MUST implement OAuth 2.0 state parameter validation to prevent CSRF attacks
- **FR-018**: System MUST securely exchange OAuth authorization codes for access tokens server-side only
- **FR-019**: System MUST extract and store Google profile data (sub, email, name) from OAuth tokens
- **FR-020**: System MUST link OAuth accounts to existing users when email matches
- **FR-021**: System MUST handle OAuth callback errors gracefully with user-friendly messaging

**Protected Resources**:

- **FR-022**: System MUST provide a mechanism to protect API endpoints, requiring valid authentication
- **FR-023**: System MUST return 401 Unauthorized for unauthenticated requests to protected endpoints
- **FR-024**: System MUST validate session tokens on every request to protected resources
- **FR-025**: System MUST deny access when session is expired or revoked

**Security Controls**:

- **FR-026**: System MUST implement rate limiting on authentication endpoints (login, signup) to prevent brute force attacks
- **FR-027**: System MUST implement rate limiting of 5 failed login attempts per email per 10 minutes
- **FR-028**: System MUST prevent timing attacks by using constant-time password comparison
- **FR-029**: System MUST validate and sanitize all user inputs (email, password, name) to prevent injection attacks
- **FR-030**: System MUST use HTTPS for all authentication and session-related requests in production

**Data Persistence**:

- **FR-031**: System MUST persist user data (id, name, email, hashed_password, created_at, updated_at) in Neon database
- **FR-032**: System MUST persist session data (id, user_id, token_hash, expires_at, created_at, last_active_at) in Neon database
- **FR-033**: System MUST persist OAuth account data (id, user_id, provider, provider_account_id, access_token, refresh_token, expires_at) in Neon database
- **FR-034**: System MUST enforce unique constraints on user email addresses
- **FR-035**: System MUST enforce unique constraints on session token hashes
- **FR-036**: System MUST enforce unique constraints on OAuth provider + provider_account_id combinations

**Frontend-Backend Separation**:

- **FR-037**: System MUST NOT expose authentication secrets, private keys, or OAuth client secrets to the frontend
- **FR-038**: System MUST perform all authentication validation, password hashing, and session creation in the backend (FastAPI)
- **FR-039**: System MUST NOT implement any authentication logic in the Docusaurus static frontend
- **FR-040**: Frontend MUST only redirect to backend authentication endpoints and receive session cookies

**Error Handling**:

- **FR-041**: System MUST return generic error messages for failed login attempts (to prevent email enumeration)
- **FR-042**: System MUST return specific validation errors for signup form fields (password strength, email format, required fields)
- **FR-043**: System MUST log all authentication events (signup, login, logout, failed attempts) for security auditing

### Key Entities

- **User**: Represents an authenticated user account. Attributes include unique identifier, name, email address (unique), hashed password (nullable for OAuth-only users), timestamps for account creation and updates, and optional email verification status.

- **Session**: Represents an active authenticated session for a user. Attributes include unique session identifier, reference to user, hashed session token, expiration timestamp, creation timestamp, last activity timestamp, optional device information (IP address, user agent), and optional revocation timestamp.

- **OAuth Account**: Represents a linked OAuth provider account for a user. Attributes include unique identifier, reference to user, provider name (e.g., "google"), provider-specific account identifier (e.g., Google sub), encrypted access token, encrypted refresh token, token expiration timestamp, and OAuth scope.

- **Verification Token** (optional for future): Represents short-lived tokens for email verification, password reset, or OAuth state validation. Attributes include unique identifier, reference to user or email, token type, hashed token value, expiration timestamp, consumption timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete email/password signup in under 60 seconds with valid credentials
- **SC-002**: Users can complete Google OAuth signup in under 30 seconds (including Google authorization)
- **SC-003**: 95% of login attempts with correct credentials succeed within 2 seconds
- **SC-004**: System handles 1000 concurrent authenticated requests without session lookup degradation
- **SC-005**: Zero plain-text passwords are stored in the database (100% hashed)
- **SC-006**: Protected API endpoints return 401 Unauthorized within 500ms for invalid sessions
- **SC-007**: Session cookies persist for 30 days (or configured TTL) without requiring re-authentication
- **SC-008**: Rate limiting successfully blocks brute force attacks after 5 failed attempts per email
- **SC-009**: OAuth state validation prevents CSRF attacks with 100% success rate in security testing
- **SC-010**: 90% of users successfully authenticate on their first attempt (measuring UX quality)
- **SC-011**: All authentication events are logged with complete audit trail (user, action, timestamp, result)
- **SC-012**: System supports multi-device sessions with independent logout capability per device

## Scope *(mandatory)*

### In Scope

- Email/password user signup and login
- Google OAuth 2.0 signup and login
- Secure session management with database-backed sessions in Neon Postgres
- Protected API endpoint authentication via session validation
- Session cookies with security attributes (HttpOnly, Secure, SameSite)
- Rate limiting on authentication endpoints
- Password hashing with bcrypt or argon2
- OAuth account linking to existing users by email
- User logout and session revocation
- Multi-device session support
- Retrieval of current authenticated user/session
- Database schema for users, sessions, and OAuth accounts in Neon
- Security audit logging for authentication events
- Input validation and sanitization for all user-provided data
- Error handling with appropriate user messaging
- CSRF protection via OAuth state parameter

### Out of Scope

- Email verification (can be added in future iteration)
- Password reset functionality (can be added in future iteration)
- Multi-factor authentication (MFA/2FA)
- Social login providers beyond Google (GitHub, Facebook, etc.)
- Magic link authentication
- Passwordless authentication (WebAuthn, passkeys)
- Account deletion or data export
- User profile editing
- Admin user management interface
- Session management UI for listing/revoking sessions per device
- Account linking UI for manually connecting OAuth providers
- JWT-based authentication (using session cookies instead)
- Refresh token rotation
- Fine-grained role-based access control (RBAC) or permissions
- Organization/team accounts
- Single sign-on (SSO) for enterprise
- IP-based geolocation or device fingerprinting

### Dependencies

- **Neon Serverless Postgres**: Database for persisting users, sessions, and OAuth accounts
- **Google OAuth 2.0**: Google Cloud Console project with OAuth credentials (Client ID, Client Secret)
- **Better Auth framework**: Authentication library and patterns (source of truth per constitution)
- **HTTPS/TLS**: Required in production for secure cookie transmission
- **FastAPI backend**: Hosts authentication endpoints and session validation
- **Docusaurus frontend**: Provides signup/login UI and redirects to backend auth endpoints
- **Environment variables**: Secure storage for OAuth secrets, database credentials, session secrets

## Assumptions *(mandatory)*

- Users have access to a valid email address for signup
- Users have a Google account if they want to use OAuth login
- The application has a valid Google OAuth 2.0 Client ID and Client Secret configured
- Neon Serverless Postgres database is provisioned and accessible from the backend
- The backend (FastAPI) has network access to Google OAuth endpoints
- HTTPS is enabled in production environments for secure cookie transmission
- Session secret keys are cryptographically secure and rotated periodically
- The frontend can make cross-origin requests to the backend (CORS configured properly)
- Rate limiting infrastructure (in-memory or Redis) is available for authentication endpoints
- Standard HTTP status codes (200, 201, 401, 403, 429, 503) are acceptable for API responses
- Users are informed via clear UI messaging when rate limiting is applied
- OAuth tokens (access_token, refresh_token) are encrypted at rest in the database
- Session token hashing uses a secure algorithm (SHA-256 or stronger)
- Password hashing work factor is balanced for security and performance (bcrypt cost 10-12)
- The application follows Better Auth recommended practices as documented
- Logging infrastructure is available for authentication audit trails
- No legacy authentication system exists that requires migration

## Constraints *(mandatory)*

- **Technology Mandate**: MUST use Better Auth as the single source of truth for authentication (per project constitution)
- **Database**: MUST use Neon Serverless Postgres for all authentication data storage
- **Backend-Only Auth Logic**: ALL authentication validation, password hashing, token generation, and session management MUST occur in FastAPI backend
- **Frontend Limitations**: Docusaurus static frontend MUST NOT contain any authentication secrets, logic, or validation
- **No Custom Auth**: MUST NOT implement custom authentication schemes or deviate from Better Auth patterns
- **Security Standards**: MUST follow OWASP authentication best practices (secure password hashing, session management, CSRF protection)
- **Architecture**: MUST maintain API-first, decoupled frontend/backend architecture
- **OAuth Provider**: Google OAuth is the ONLY social login provider in initial release
- **Session Storage**: MUST use database-backed sessions (NOT stateless JWT-only)
- **Cookie-Based Sessions**: MUST use HTTP cookies for session token storage (with proper security flags)
- **No Blocking Operations**: Authentication endpoints MUST NOT perform long-running operations (keep response times under 3 seconds)
- **Existing System Integration**: MUST NOT disrupt or break existing RAG chatbot functionality
- **Migration**: No migration from existing auth system required (net-new implementation)

## Authentication Flows *(mandatory)*

### Email/Password Signup Flow

1. User navigates to signup page on Docusaurus frontend
2. User enters name, email, and password into signup form
3. Frontend validates basic input format (email format, password length) client-side
4. Frontend sends POST request to backend `/api/auth/register` with { name, email, password }
5. Backend validates inputs:
   - Email format and uniqueness
   - Password strength (min 8 characters, complexity rules)
   - Name is non-empty
6. Backend hashes password using bcrypt/argon2
7. Backend creates user record in Neon `users` table
8. Backend creates session record in Neon `sessions` table with hashed token
9. Backend sets HttpOnly, Secure, SameSite session cookie in response
10. Backend returns 201 Created with user data (id, name, email) and session status
11. Frontend redirects user to authenticated application home page

**Error Paths**:
- Email already exists → 409 Conflict with message "Email already registered"
- Invalid email format → 400 Bad Request with validation error
- Weak password → 400 Bad Request with password requirements
- Database error → 503 Service Unavailable

### Email/Password Login Flow

1. User navigates to login page on Docusaurus frontend
2. User enters email and password into login form
3. Frontend sends POST request to backend `/api/auth/login` with { email, password }
4. Backend checks rate limiting for this email (max 5 attempts per 10 minutes)
5. Backend retrieves user record from Neon `users` table by email
6. Backend verifies password hash using constant-time comparison
7. Backend creates new session record in Neon `sessions` table
8. Backend sets HttpOnly, Secure, SameSite session cookie with session token
9. Backend returns 200 OK with user data (id, name, email) and session status
10. Frontend redirects user to authenticated application home page

**Error Paths**:
- Invalid credentials (wrong password or email) → 401 Unauthorized with generic message "Invalid email or password"
- Rate limit exceeded → 429 Too Many Requests with retry-after guidance
- Database error → 503 Service Unavailable

### Google OAuth Signup/Login Flow

1. User clicks "Sign up with Google" or "Login with Google" button on frontend
2. Frontend redirects to backend `/api/auth/oauth/google`
3. Backend generates cryptographically secure state parameter
4. Backend stores state in session or short-lived token
5. Backend redirects user to Google OAuth authorization endpoint with:
   - Client ID
   - Redirect URI (backend callback URL)
   - Scope (email, profile, openid)
   - State parameter
6. User authorizes application at Google
7. Google redirects to backend callback `/api/auth/oauth/google/callback` with authorization code and state
8. Backend validates state parameter to prevent CSRF
9. Backend exchanges authorization code for tokens (access_token, id_token, refresh_token) via Google Token endpoint
10. Backend extracts user profile data from id_token (Google sub, email, name)
11. Backend checks if OAuth account exists:
    - **If exists**: Retrieve linked user
    - **If not exists, but email matches existing user**: Link OAuth account to existing user
    - **If not exists**: Create new user record and OAuth account record
12. Backend creates session record in Neon `sessions` table
13. Backend sets HttpOnly, Secure, SameSite session cookie
14. Backend redirects user to authenticated application home page
15. Frontend receives user in authenticated state

**Error Paths**:
- User cancels OAuth at Google → Redirect to frontend with error parameter
- Invalid or expired state → 400 Bad Request with security error
- OAuth token exchange fails → 503 Service Unavailable with retry guidance
- Database error → 503 Service Unavailable

### Session Validation Flow (Protected Endpoints)

1. Authenticated user makes request to protected API endpoint (e.g., `/api/chatbot/history`)
2. Backend reads session token from HttpOnly cookie
3. Backend hashes token and queries Neon `sessions` table by token_hash
4. Backend validates session:
   - Session exists
   - Not expired (expires_at > current time)
   - Not revoked (revoked_at is null)
5. Backend retrieves user record via session.user_id
6. Backend updates session.last_active_at (sliding expiration)
7. Backend attaches user to request context
8. Protected endpoint handler proceeds with authenticated user

**Error Paths**:
- No session cookie → 401 Unauthorized
- Invalid session token → 401 Unauthorized
- Session expired → 401 Unauthorized with "Session expired" message
- Session revoked → 401 Unauthorized with "Session invalid" message
- Database error → 503 Service Unavailable

### Logout Flow

1. Authenticated user clicks logout button on frontend
2. Frontend sends POST request to backend `/api/auth/logout`
3. Backend reads session token from cookie
4. Backend marks session as revoked in Neon `sessions` table (set revoked_at = now)
5. Backend clears session cookie (Set-Cookie with Max-Age=0)
6. Backend returns 200 OK
7. Frontend redirects user to login page

**Error Paths**:
- No active session → 200 OK (idempotent operation)
- Database error → 503 Service Unavailable (session may not be revoked)

## Database Schema *(mandatory)*

### Users Table

**Purpose**: Store user account information for both email/password and OAuth users.

**Columns**:
- `id` (UUID, Primary Key): Unique user identifier
- `email` (VARCHAR(255), UNIQUE, NOT NULL): User's email address (unique across system)
- `name` (VARCHAR(255), NOT NULL): User's display name
- `hashed_password` (VARCHAR(255), NULLABLE): Bcrypt/argon2 hashed password (null for OAuth-only users)
- `email_verified` (BOOLEAN, DEFAULT FALSE): Email verification status (for future use)
- `created_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): Account creation timestamp
- `updated_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): Last account update timestamp

**Indexes**:
- Primary key on `id`
- Unique index on `email`
- Index on `created_at` for reporting

**Constraints**:
- Email must be valid format (application-level validation)
- Password must be hashed before storage (never plain-text)

### Sessions Table

**Purpose**: Store active user sessions for authentication.

**Columns**:
- `id` (UUID, Primary Key): Unique session identifier
- `user_id` (UUID, NOT NULL, Foreign Key → users.id): Reference to user
- `token_hash` (VARCHAR(255), UNIQUE, NOT NULL): Hashed session token (SHA-256 or stronger)
- `expires_at` (TIMESTAMP, NOT NULL): Session expiration timestamp (default 30 days from creation)
- `created_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): Session creation timestamp
- `last_active_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): Last activity timestamp (updated on requests)
- `revoked_at` (TIMESTAMP, NULLABLE): Session revocation timestamp (null = active)
- `ip_address` (VARCHAR(45), NULLABLE): IP address of session creation (for audit/security)
- `user_agent` (TEXT, NULLABLE): Browser/device user agent (for device management in future)

**Indexes**:
- Primary key on `id`
- Unique index on `token_hash`
- Index on `user_id` (for user session lookup)
- Index on `expires_at` (for cleanup queries)

**Constraints**:
- Foreign key `user_id` references `users.id` ON DELETE CASCADE
- Token hash must be unique (prevent duplicates)
- Expires_at must be in the future at creation

### OAuth Accounts Table

**Purpose**: Store linked OAuth provider accounts (Google) for users.

**Columns**:
- `id` (UUID, Primary Key): Unique OAuth account identifier
- `user_id` (UUID, NOT NULL, Foreign Key → users.id): Reference to user
- `provider` (VARCHAR(50), NOT NULL): OAuth provider name (e.g., "google")
- `provider_account_id` (VARCHAR(255), NOT NULL): Provider-specific user identifier (e.g., Google sub)
- `access_token` (TEXT, NULLABLE): Encrypted OAuth access token
- `refresh_token` (TEXT, NULLABLE): Encrypted OAuth refresh token
- `expires_at` (TIMESTAMP, NULLABLE): Access token expiration timestamp
- `scope` (VARCHAR(255), NULLABLE): OAuth scopes granted
- `token_type` (VARCHAR(50), NULLABLE): Token type (e.g., "Bearer")
- `created_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): OAuth account link timestamp
- `updated_at` (TIMESTAMP, NOT NULL, DEFAULT NOW()): Last token refresh timestamp

**Indexes**:
- Primary key on `id`
- Unique composite index on `(provider, provider_account_id)`
- Index on `user_id` (for user OAuth account lookup)

**Constraints**:
- Foreign key `user_id` references `users.id` ON DELETE CASCADE
- Unique constraint on `(provider, provider_account_id)` prevents duplicate OAuth links
- Tokens must be encrypted at rest

## API Contracts *(mandatory)*

### POST /api/auth/register

**Purpose**: Create a new user account with email and password.

**Request**:
- Method: POST
- Headers: `Content-Type: application/json`
- Body:
  ```json
  {
    "name": "string (required, 1-255 chars)",
    "email": "string (required, valid email format)",
    "password": "string (required, min 8 chars)"
  }
  ```

**Response**:
- Success (201 Created):
  ```json
  {
    "user": {
      "id": "uuid",
      "name": "string",
      "email": "string",
      "created_at": "ISO 8601 timestamp"
    },
    "session": {
      "id": "uuid",
      "expires_at": "ISO 8601 timestamp"
    }
  }
  ```
  - Sets `Set-Cookie` header with session token (HttpOnly, Secure, SameSite)

- Error (400 Bad Request - validation):
  ```json
  {
    "error": "Validation failed",
    "details": {
      "password": "Password must be at least 8 characters"
    }
  }
  ```

- Error (409 Conflict - duplicate email):
  ```json
  {
    "error": "Email already registered"
  }
  ```

- Error (429 Too Many Requests - rate limit):
  ```json
  {
    "error": "Too many signup attempts. Please try again in 10 minutes."
  }
  ```

---

### POST /api/auth/login

**Purpose**: Authenticate user with email and password.

**Request**:
- Method: POST
- Headers: `Content-Type: application/json`
- Body:
  ```json
  {
    "email": "string (required)",
    "password": "string (required)"
  }
  ```

**Response**:
- Success (200 OK):
  ```json
  {
    "user": {
      "id": "uuid",
      "name": "string",
      "email": "string"
    },
    "session": {
      "id": "uuid",
      "expires_at": "ISO 8601 timestamp"
    }
  }
  ```
  - Sets `Set-Cookie` header with session token

- Error (401 Unauthorized - invalid credentials):
  ```json
  {
    "error": "Invalid email or password"
  }
  ```
  - Note: Generic message to prevent email enumeration

- Error (429 Too Many Requests - rate limit):
  ```json
  {
    "error": "Too many login attempts. Please try again in 10 minutes.",
    "retry_after": 600
  }
  ```

---

### POST /api/auth/logout

**Purpose**: Terminate current user session.

**Request**:
- Method: POST
- Headers: Session cookie (sent automatically by browser)

**Response**:
- Success (200 OK):
  ```json
  {
    "message": "Logged out successfully"
  }
  ```
  - Sets `Set-Cookie` header with Max-Age=0 to clear cookie

- Error (401 Unauthorized - no session):
  ```json
  {
    "error": "No active session"
  }
  ```
  - Note: This is also treated as success (idempotent)

---

### GET /api/auth/oauth/google

**Purpose**: Initiate Google OAuth flow.

**Request**:
- Method: GET
- Query Parameters: None

**Response**:
- Success (302 Redirect):
  - Redirects to Google OAuth authorization endpoint with state parameter

---

### GET /api/auth/oauth/google/callback

**Purpose**: Handle Google OAuth callback and create session.

**Request**:
- Method: GET
- Query Parameters:
  - `code`: OAuth authorization code (from Google)
  - `state`: CSRF protection state parameter

**Response**:
- Success (302 Redirect):
  - Redirects to authenticated application home page
  - Sets session cookie

- Error (400 Bad Request - invalid state):
  ```json
  {
    "error": "Invalid or expired OAuth state"
  }
  ```

- Error (400 Bad Request - user cancelled):
  - Redirects to login page with `error=access_denied` query parameter

---

### GET /api/auth/session

**Purpose**: Retrieve current authenticated user and session.

**Request**:
- Method: GET
- Headers: Session cookie (sent automatically by browser)

**Response**:
- Success (200 OK - authenticated):
  ```json
  {
    "user": {
      "id": "uuid",
      "name": "string",
      "email": "string"
    },
    "session": {
      "id": "uuid",
      "expires_at": "ISO 8601 timestamp",
      "last_active_at": "ISO 8601 timestamp"
    }
  }
  ```

- Success (200 OK - not authenticated):
  ```json
  {
    "user": null,
    "session": null
  }
  ```

---

### Protected Endpoint Pattern (Example: GET /api/chatbot/history)

**Purpose**: Any protected resource requiring authentication.

**Request**:
- Method: GET/POST/etc.
- Headers: Session cookie (sent automatically by browser)

**Response**:
- Success (200 OK - authenticated):
  - Returns requested resource data

- Error (401 Unauthorized - not authenticated):
  ```json
  {
    "error": "Authentication required",
    "message": "Please log in to access this resource"
  }
  ```

- Error (401 Unauthorized - session expired):
  ```json
  {
    "error": "Session expired",
    "message": "Your session has expired. Please log in again."
  }
  ```

## Security Considerations *(mandatory)*

### Password Security
- All passwords MUST be hashed using bcrypt (cost factor 10-12) or argon2 before storage
- Password comparison MUST use constant-time algorithms to prevent timing attacks
- Minimum password requirements: 8 characters, combination of letters and numbers
- Plain-text passwords MUST NEVER be logged, stored, or transmitted except during initial submission over HTTPS

### Session Security
- Session tokens MUST be cryptographically random (minimum 128 bits entropy)
- Session tokens MUST be hashed before storage in database (SHA-256 or stronger)
- Session cookies MUST use HttpOnly flag (prevent JavaScript access)
- Session cookies MUST use Secure flag in production (HTTPS only)
- Session cookies MUST use SameSite=Lax or Strict (CSRF protection)
- Session expiration MUST be enforced on every request
- Revoked sessions MUST be rejected immediately

### OAuth Security
- OAuth state parameter MUST be cryptographically random and validated on callback
- OAuth code exchange MUST happen server-side only (never expose client secret to frontend)
- OAuth tokens (access_token, refresh_token) MUST be encrypted at rest
- OAuth redirect URIs MUST be whitelisted and validated

### Rate Limiting
- Authentication endpoints MUST implement rate limiting: 5 failed attempts per email per 10 minutes
- Rate limit counters MUST reset on successful authentication
- Rate limit errors MUST return 429 Too Many Requests with retry-after guidance

### Input Validation
- All user inputs (email, password, name) MUST be validated and sanitized
- Email format MUST be validated using standard regex patterns
- SQL injection prevention via parameterized queries (ORM usage)
- XSS prevention via output encoding (handled by framework defaults)

### HTTPS/TLS
- All authentication and session requests MUST use HTTPS in production
- Secure cookie flag MUST be enabled in production environments
- HTTP Strict Transport Security (HSTS) headers recommended

### Audit Logging
- All authentication events MUST be logged: signup, login, logout, failed attempts
- Logs MUST include: timestamp, user identifier (email or ID), action, IP address, result (success/failure)
- Logs MUST NOT include passwords, tokens, or sensitive data
- Failed login attempts MUST be logged for security monitoring

### Frontend Security
- Authentication secrets (OAuth client secret, session keys) MUST NEVER be exposed to frontend
- All authentication logic MUST execute in backend only
- Frontend MUST only redirect to backend auth endpoints and receive cookies
- CORS MUST be configured to allow only trusted frontend origins

### Database Security
- Database credentials MUST be stored in environment variables (never in code)
- Database connections MUST use TLS/SSL encryption
- Principle of least privilege: Auth service account should only access auth tables
- Regular backups of user and session data for disaster recovery

## Non-Functional Requirements *(mandatory)*

### Performance
- Authentication endpoints MUST respond within 2 seconds for 95% of requests
- Session validation MUST complete within 500ms for protected endpoints
- Database queries MUST use indexes for email, token_hash, user_id lookups
- Password hashing work factor MUST balance security and performance (bcrypt cost 10-12)

### Scalability
- System MUST support 1000 concurrent authenticated users without degradation
- Session storage MUST be optimized for high-frequency reads (consider caching layer in future)
- Database connection pooling MUST be configured for FastAPI concurrency

### Availability
- Authentication service MUST have 99.5% uptime (allowing for planned maintenance)
- Graceful degradation: If OAuth provider is down, email/password login remains available
- Database connection failures MUST return 503 Service Unavailable with retry guidance

### Maintainability
- Authentication code MUST follow Better Auth patterns and conventions
- Database migrations MUST be versioned and reversible
- Configuration MUST use environment variables (12-factor app methodology)
- Code MUST include inline documentation for authentication flows

### Compliance
- Password storage MUST comply with OWASP password storage guidelines
- Session management MUST comply with OWASP session management best practices
- OAuth implementation MUST comply with RFC 6749 (OAuth 2.0 specification)
- GDPR considerations: User data retention and deletion (future scope)

### Compatibility
- Frontend MUST work with modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)
- Backend MUST support HTTP/1.1 and HTTP/2
- Database schema MUST be compatible with Neon Serverless Postgres (PostgreSQL 15+)

## Risks and Mitigations *(mandatory)*

### Risk 1: OAuth Provider Outage
- **Probability**: Low
- **Impact**: High (blocks Google login/signup)
- **Mitigation**: Provide email/password as fallback authentication method; display clear error messaging when OAuth is unavailable
- **Monitoring**: Alert on repeated OAuth failures; track OAuth vs email/password signup ratio

### Risk 2: Session Token Theft
- **Probability**: Low (with HTTPS and HttpOnly cookies)
- **Impact**: High (account takeover)
- **Mitigation**: Use HttpOnly, Secure, SameSite cookies; enforce HTTPS; implement session revocation; log suspicious activity
- **Monitoring**: Track session access from different IPs/devices; provide user-facing session management UI (future)

### Risk 3: Brute Force Password Attacks
- **Probability**: Medium
- **Impact**: Medium (account compromise)
- **Mitigation**: Implement rate limiting (5 attempts per 10 minutes); use strong password requirements; consider account lockout after N failed attempts (future)
- **Monitoring**: Alert on high volume of failed login attempts; track failed attempt patterns

### Risk 4: Email Enumeration
- **Probability**: Medium
- **Impact**: Low (reveals registered emails)
- **Mitigation**: Use generic error messages for failed login; return same response time for valid and invalid emails (constant-time)
- **Monitoring**: Track signup patterns for abusive enumeration attempts

### Risk 5: Database Performance Degradation
- **Probability**: Medium (as user base grows)
- **Impact**: High (slow authentication)
- **Mitigation**: Use indexes on high-query columns (email, token_hash, user_id); implement query optimization; consider read replicas (future); monitor query performance
- **Monitoring**: Track database query times; alert on slow queries; capacity planning for user growth

### Risk 6: OAuth Account Linking Conflicts
- **Probability**: Low
- **Impact**: Medium (user confusion, support burden)
- **Mitigation**: Clear UX when linking OAuth to existing account; document expected behavior; provide account unlinking (future)
- **Monitoring**: Track account linking events; monitor support tickets related to OAuth

### Risk 7: Session Fixation Attacks
- **Probability**: Low
- **Impact**: High (session hijacking)
- **Mitigation**: Rotate session tokens on login; never accept session tokens from URL parameters; use framework-provided session management
- **Monitoring**: Not directly monitorable; rely on security testing

### Risk 8: Password Database Breach
- **Probability**: Very Low
- **Impact**: High (mass credential leak)
- **Mitigation**: Use strong password hashing (bcrypt/argon2); encrypt database at rest; limit database access; regular security audits
- **Monitoring**: Database access logging; regular security assessments; password hash algorithm strength reviews

## Open Questions *(if applicable)*

None. All requirements are sufficiently defined for planning phase.

## Related Documentation *(if applicable)*

- Better Auth Documentation: (to be referenced during planning)
- Neon Database Documentation: https://neon.tech/docs
- OAuth 2.0 Specification: RFC 6749
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
- OWASP Session Management Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Session_Management_Cheat_Sheet.html
- Project Constitution: `.specify/memory/constitution.md`
