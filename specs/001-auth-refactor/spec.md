# Feature Specification: Authentication Refactor & Integration

**Feature Branch**: `001-auth-refactor`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Create a detailed specification to refactor and fix authentication integration in my existing project."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Unified Backend Architecture (Priority: P1)

A developer needs to run a single FastAPI backend server that handles both RAG chatbot and authentication functionalities without managing multiple server processes or dealing with port conflicts and redirect issues.

**Why this priority**: This is foundational infrastructure work that unblocks all other user-facing features. Without a properly unified backend, authentication flows cannot work correctly, making all subsequent user stories impossible.

**Independent Test**: Can be fully tested by starting the application and verifying that:
- Only one server process runs on a single port (localhost:8000)
- Both `/api/*` (RAG chatbot) and `/auth/*` (authentication) endpoints are accessible
- No redirect errors occur between services
- Development workflow uses one command to start the entire backend

**Acceptance Scenarios**:

1. **Given** the backend codebase with separate auth and RAG modules, **When** a developer starts the application, **Then** a single FastAPI server starts on port 8000 serving both `/api/*` and `/auth/*` routes
2. **Given** the unified backend is running, **When** a request is made to `/api/health`, **Then** the RAG backend responds successfully
3. **Given** the unified backend is running, **When** a request is made to `/auth/login`, **Then** the auth backend responds successfully without redirect errors
4. **Given** the auth backend was previously in `rag-chatbot/backend/auth_backend`, **When** the directory structure is refactored, **Then** auth backend becomes a top-level module at `rag-chatbot/auth_backend`

---

### User Story 2 - User Registration and Login (Priority: P2)

A new user visits the application and wants to create an account to access the RAG chatbot. They should be able to register with email/password or use OAuth (Google/GitHub), log in successfully, and maintain their authenticated session.

**Why this priority**: This is the entry point for all users. Without working registration and login, no one can access the application, making it the highest priority user-facing feature.

**Independent Test**: Can be fully tested by:
- Registering a new account via email/password form
- Logging in with those credentials
- Registering/logging in via OAuth provider
- Verifying session persistence across page refreshes
- Confirming proper error messages for invalid credentials

**Acceptance Scenarios**:

1. **Given** a user visits the registration page, **When** they submit valid email, password, and confirm password, **Then** an account is created and they are automatically logged in
2. **Given** a user visits the login page, **When** they submit valid credentials, **Then** they are authenticated and redirected appropriately
3. **Given** a user chooses OAuth login, **When** they complete the OAuth flow with Google or GitHub, **Then** they are authenticated and session is established
4. **Given** a user enters invalid credentials, **When** they submit the login form, **Then** they see a clear error message explaining the issue
5. **Given** a user enters mismatched passwords during registration, **When** they submit the form, **Then** they see a validation error before account creation
6. **Given** a user is authenticated, **When** they refresh the page, **Then** their session persists and they remain logged in

---

### User Story 3 - Mandatory Onboarding Flow (Priority: P2)

A newly registered user must complete an onboarding questionnaire before accessing the RAG chatbot. This collects their user type, interests, and experience level to personalize their experience.

**Why this priority**: This is required immediately after registration to collect essential user profile data. It must happen before chatbot access but after authentication, making it critical to the user journey.

**Independent Test**: Can be fully tested by:
- Registering a new account
- Verifying redirect to onboarding page (not directly to chatbot)
- Completing the onboarding form with all required fields
- Confirming successful save to database
- Verifying redirect to application after completion
- Attempting to access chatbot before onboarding (should be blocked)

**Acceptance Scenarios**:

1. **Given** a user just registered or logged in for the first time, **When** authentication succeeds, **Then** they are redirected to the onboarding page (not the chatbot)
2. **Given** a user is on the onboarding page, **When** they view the form, **Then** they see fields for: user type (Student/Researcher/Teacher/Engineer/etc.), area of interest, experience level (Beginner/Intermediate/Advanced), and topics of interest (optional: ROS2/NVIDIA Isaac/Voice Control/LLM Integration/Reinforcement Learning)
3. **Given** a user completes the onboarding form, **When** they submit it, **Then** their profile data is saved to the database linked to their user account
4. **Given** a user successfully completes onboarding, **When** the save operation succeeds, **Then** they are redirected to the base application URL (chatbot access)
5. **Given** a user has not completed onboarding, **When** they attempt to access the chatbot directly, **Then** they are redirected back to the onboarding page
6. **Given** a user has already completed onboarding, **When** they log in again, **Then** they skip the onboarding page and go directly to the application

---

### User Story 4 - Protected RAG Chatbot Access (Priority: P3)

An authenticated and onboarded user accesses the RAG chatbot to ask questions about Physical AI and Humanoid Robotics. Unauthenticated users are prevented from accessing the chat interface.

**Why this priority**: This enforces the security requirement that only authenticated users can use the chatbot. It depends on P1 (unified backend), P2 (registration/login), and P3 (onboarding) being complete.

**Independent Test**: Can be fully tested by:
- Logging in as an authenticated user who completed onboarding
- Accessing the chatbot and successfully sending messages
- Logging out and attempting to access chatbot (should redirect to login)
- Verifying existing RAG functionality remains unchanged

**Acceptance Scenarios**:

1. **Given** an unauthenticated user, **When** they attempt to access the chatbot page, **Then** they are redirected to the login page
2. **Given** an authenticated user who has completed onboarding, **When** they access the chatbot page, **Then** they can view and use the chat interface
3. **Given** an authenticated user who has NOT completed onboarding, **When** they attempt to access the chatbot, **Then** they are redirected to the onboarding page
4. **Given** an authenticated and onboarded user using the chatbot, **When** they send a message, **Then** the RAG backend processes it and returns relevant responses (existing functionality preserved)
5. **Given** a user's session expires, **When** they attempt to send a chat message, **Then** they are redirected to login

---

### User Story 5 - Session Management and Logout (Priority: P4)

An authenticated user wants to securely log out of the application, ending their session and requiring re-authentication to access protected resources.

**Why this priority**: While important for security, this is lower priority than getting users into the system. Users can still use the application without explicit logout functionality in early testing.

**Independent Test**: Can be fully tested by:
- Logging in as a user
- Clicking logout button
- Verifying session is destroyed
- Confirming redirect to login page
- Attempting to access protected resources (should fail)

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they click the logout button, **Then** their session is destroyed and they are redirected to the login page
2. **Given** a user just logged out, **When** they attempt to access protected routes, **Then** they are redirected to login
3. **Given** a user just logged out, **When** they check their session storage/cookies, **Then** authentication tokens are cleared

---

### Edge Cases

- **What happens when** a user's session expires while they are actively using the chatbot?
  - Expected: Graceful session expiration with redirect to login, preserving unsent message (if possible) or showing clear notification

- **What happens when** OAuth provider (Google/GitHub) is temporarily unavailable during login?
  - Expected: Clear error message indicating provider unavailability, option to use email/password instead

- **What happens when** a user abandons the onboarding form halfway through?
  - Expected: Form data is not saved, user can return later and must complete from the beginning, still cannot access chatbot until completion

- **What happens when** a user registers with an email that already exists?
  - Expected: Clear error message indicating email is already registered, suggestion to log in or reset password

- **What happens when** the backend database is temporarily unreachable?
  - Expected: Graceful error handling with user-friendly message, no application crash, retry mechanism or maintenance mode

- **What happens when** a user tries to access `/api/*` endpoints directly via API calls without authentication?
  - Expected: 401 Unauthorized response with proper error message

- **What happens when** frontend makes concurrent requests during authentication (race condition)?
  - Expected: Proper synchronization of auth state, no duplicate session creation

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST serve all backend functionality from a single FastAPI application instance on a single port (e.g., localhost:8000)
- **FR-002**: System MUST route RAG chatbot requests to `/api/*` endpoints
- **FR-003**: System MUST route authentication requests to `/auth/*` endpoints
- **FR-004**: System MUST organize the codebase with `rag-chatbot/backend` for RAG functionality and `rag-chatbot/auth_backend` for authentication functionality as separate logical modules
- **FR-005**: System MUST support user registration via email/password with validation
- **FR-006**: System MUST support user login via email/password
- **FR-007**: System MUST support OAuth authentication via Google and GitHub
- **FR-008**: System MUST handle OAuth redirects correctly without port conflicts or cross-origin issues
- **FR-009**: System MUST automatically log in users after successful registration
- **FR-010**: System MUST create and manage user sessions with secure tokens
- **FR-011**: System MUST enforce authentication for all `/api/*` endpoints (except public health checks)
- **FR-012**: System MUST redirect unauthenticated users attempting to access protected resources to the login page
- **FR-013**: System MUST present an onboarding form to newly registered users immediately after authentication
- **FR-014**: System MUST collect the following onboarding data: user type (predefined options: Student, Researcher, Teacher, Engineer, Other), area of interest (predefined options), experience level (Beginner, Intermediate, Advanced), topics of interest (optional multi-select: ROS2, NVIDIA Isaac, Voice Control, LLM Integration, Reinforcement Learning)
- **FR-015**: System MUST store onboarding data in the database (Neon Serverless Postgres) linked to the user's account
- **FR-016**: System MUST prevent access to the RAG chatbot until onboarding is completed
- **FR-017**: System MUST skip the onboarding flow for returning users who have already completed it
- **FR-018**: System MUST redirect authenticated and onboarded users to the base application URL (chatbot interface)
- **FR-019**: System MUST preserve existing RAG chatbot functionality without modification
- **FR-020**: System MUST allow users to log out, destroying their session
- **FR-021**: System MUST maintain session state across page refreshes and navigation
- **FR-022**: System MUST provide clear, user-friendly error messages for authentication failures
- **FR-023**: System MUST validate onboarding form inputs before saving
- **FR-024**: System MUST support the existing Docusaurus frontend without requiring frontend restructuring

### Key Entities

- **User**: Represents an authenticated user account
  - Attributes: unique identifier, email, password hash (for email/password auth), OAuth provider info (for OAuth users), registration timestamp, last login timestamp, onboarding completion status
  - Relationships: Has one onboarding profile, has many chat sessions

- **Onboarding Profile**: Represents user profile data collected during onboarding
  - Attributes: user reference, user type, area of interest, experience level, topics of interest (list), completion timestamp
  - Relationships: Belongs to one user

- **Session**: Represents an active user authentication session
  - Attributes: session token, user reference, creation timestamp, expiration timestamp, refresh token
  - Relationships: Belongs to one user

- **OAuth Account**: Represents a linked OAuth provider account
  - Attributes: user reference, provider name (Google/GitHub), provider user ID, access token, refresh token
  - Relationships: Belongs to one user

- **Conversation/Chat Session**: Existing RAG chatbot entity (unchanged)
  - Attributes: session ID, user reference, messages, timestamps
  - Relationships: Belongs to one user

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Developers can start the entire backend with a single command, resulting in one server process on one port
- **SC-002**: Users can successfully register an account in under 2 minutes
- **SC-003**: Users can complete the onboarding form in under 3 minutes
- **SC-004**: 100% of unauthenticated access attempts to the chatbot are redirected to login
- **SC-005**: OAuth authentication flow completes successfully without redirect errors
- **SC-006**: Authenticated sessions persist across browser refreshes and tab navigation
- **SC-007**: All existing RAG chatbot functionality continues to work without regression
- **SC-008**: API endpoints respond correctly to authentication state (401 for unauthenticated, 200 for authenticated with valid permissions)
- **SC-009**: User onboarding data is successfully saved to the database with 100% accuracy
- **SC-010**: Login success rate reaches 95%+ for valid credentials (accounting for user typos)
- **SC-011**: The system handles 100 concurrent authentication requests without performance degradation
- **SC-012**: Frontend receives clear authentication state (authenticated/unauthenticated/onboarding-required) to render appropriate UI

## Architecture Overview

### Directory Structure

**Current Structure (Before)**:
```
rag-chatbot/
├── backend/
│   ├── app/
│   │   ├── api/          # RAG API endpoints
│   │   ├── models/       # RAG database models
│   │   └── services/     # RAG business logic
│   ├── auth_backend/     # ❌ INCORRECTLY NESTED
│   │   ├── api/
│   │   ├── models/
│   │   └── services/
│   └── main.py           # RAG server entry point
└── auth_backend/         # ❌ DUPLICATE/ORPHANED
    └── run.py            # Separate auth server (problematic)
```

**Target Structure (After)**:
```
rag-chatbot/
├── backend/              # RAG Chatbot Backend
│   ├── app/
│   │   ├── api/          # RAG API endpoints
│   │   ├── models/       # RAG database models
│   │   └── services/     # RAG business logic
│   └── main.py           # 🎯 UNIFIED server entry point
│
├── auth_backend/         # ✅ TOP-LEVEL Auth Backend Module
│   ├── api/
│   │   ├── routes/
│   │   │   ├── auth.py         # Login, register, logout
│   │   │   ├── oauth.py        # OAuth flows
│   │   │   └── onboarding.py   # Onboarding endpoints
│   │   └── deps.py             # Auth dependencies (session verification)
│   ├── models/
│   │   ├── user.py
│   │   ├── session.py
│   │   ├── oauth_account.py
│   │   └── onboarding_profile.py  # 🆕 NEW
│   ├── services/
│   │   ├── password.py
│   │   ├── session.py
│   │   ├── oauth.py
│   │   └── onboarding.py      # 🆕 NEW
│   └── database.py
│
└── shared/               # 🆕 OPTIONAL: Shared utilities
    ├── middleware/       # Auth middleware for RAG endpoints
    └── config/           # Shared configuration
```

### API Routes

All routes served from **localhost:8000**:

**Authentication Routes** (`/auth/*`):
- `POST /auth/register` - Create new user account (email/password)
- `POST /auth/login` - Authenticate user (email/password)
- `POST /auth/logout` - Destroy user session
- `GET /auth/me` - Get current authenticated user info
- `GET /auth/oauth/google` - Initiate Google OAuth flow
- `GET /auth/oauth/google/callback` - Handle Google OAuth redirect
- `GET /auth/oauth/github` - Initiate GitHub OAuth flow
- `GET /auth/oauth/github/callback` - Handle GitHub OAuth redirect
- `POST /auth/refresh` - Refresh authentication token

**Onboarding Routes** (`/auth/onboarding/*`):
- `GET /auth/onboarding/status` - Check if user has completed onboarding
- `GET /auth/onboarding/options` - Get predefined options for form fields
- `POST /auth/onboarding/complete` - Submit onboarding form data
- `GET /auth/onboarding/profile` - Get user's onboarding profile

**RAG Chatbot Routes** (`/api/*`) - ALL REQUIRE AUTHENTICATION:
- `GET /api/health` - Health check (public, no auth required)
- `POST /api/chat` - Send message to RAG chatbot
- `GET /api/sessions` - Get user's chat sessions
- `POST /api/sessions` - Create new chat session
- `DELETE /api/sessions/{id}` - Delete chat session
- (All existing RAG endpoints preserved)

### Authentication Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      User Journey Flow                           │
└─────────────────────────────────────────────────────────────────┘

1. NEW USER - REGISTRATION FLOW
   ┌──────────────┐
   │ Visit Site   │
   └──────┬───────┘
          │
          ▼
   ┌──────────────────┐      ┌─────────────────┐
   │ Click "Sign Up"  │─────▶│ Registration    │
   └──────────────────┘      │ Page            │
                             └────────┬────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                 │
                    ▼                 ▼                 ▼
          ┌─────────────────┐ ┌──────────┐  ┌──────────────┐
          │ Email/Password  │ │  Google  │  │   GitHub     │
          │      Form       │ │  OAuth   │  │   OAuth      │
          └────────┬────────┘ └─────┬────┘  └──────┬───────┘
                   │                │               │
                   └────────────────┼───────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │ Account Created  │
                          │ (Auto Login)     │
                          └────────┬─────────┘
                                   │
                                   ▼
                          ┌──────────────────┐
                          │   Onboarding     │◀─── MANDATORY
                          │      Form        │
                          └────────┬─────────┘
                                   │
                        ┌──────────┼──────────┐
                        │          │          │
                        ▼          ▼          ▼
                   [User Type] [Interest] [Experience]
                   [Topics (optional)]
                                   │
                                   ▼
                          ┌──────────────────┐
                          │  Save Profile    │
                          │   to Database    │
                          └────────┬─────────┘
                                   │
                                   ▼
                          ┌──────────────────┐
                          │   RAG Chatbot    │
                          │     Unlocked     │
                          └──────────────────┘

2. RETURNING USER - LOGIN FLOW
   ┌──────────────┐
   │ Visit Site   │
   └──────┬───────┘
          │
          ▼
   ┌──────────────────┐      ┌─────────────────┐
   │ Click "Login"    │─────▶│   Login Page    │
   └──────────────────┘      └────────┬────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                 │
                    ▼                 ▼                 ▼
          ┌─────────────────┐ ┌──────────┐  ┌──────────────┐
          │ Email/Password  │ │  Google  │  │   GitHub     │
          └────────┬────────┘ └─────┬────┘  └──────┬───────┘
                   │                │               │
                   └────────────────┼───────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │  Authenticated   │
                          └────────┬─────────┘
                                   │
                        ┌──────────┴──────────┐
                        │                     │
            Onboarding Complete?              │
                        │                     │
                  ┌─────┴─────┐               │
                  │    NO     │               │ YES
                  └─────┬─────┘               │
                        │                     │
                        ▼                     │
              ┌──────────────────┐            │
              │   Onboarding     │            │
              │      Form        │            │
              └────────┬─────────┘            │
                       │                      │
                       └──────────┬───────────┘
                                  │
                                  ▼
                          ┌──────────────────┐
                          │   RAG Chatbot    │
                          └──────────────────┘

3. PROTECTED RESOURCE ACCESS
   ┌──────────────────┐
   │ Request /api/*   │
   └──────┬───────────┘
          │
          ▼
   ┌──────────────────────┐
   │ Check Session/Token  │
   └──────┬───────────────┘
          │
    ┌─────┴─────┐
    │           │
    ▼           ▼
 [VALID]    [INVALID]
    │           │
    │           ▼
    │    ┌──────────────┐
    │    │ Redirect to  │
    │    │    Login     │
    │    └──────────────┘
    │
    ▼
 ┌──────────────────────┐
 │ Check Onboarding     │
 └──────┬───────────────┘
        │
   ┌────┴────┐
   │         │
   ▼         ▼
[DONE]   [PENDING]
   │         │
   │         ▼
   │  ┌──────────────┐
   │  │ Redirect to  │
   │  │  Onboarding  │
   │  └──────────────┘
   │
   ▼
┌──────────────┐
│ Grant Access │
└──────────────┘
```

### Database Changes

**New Table: `onboarding_profiles`**
```
Columns:
- id (UUID, primary key)
- user_id (UUID, foreign key -> users.id, unique)
- user_type (VARCHAR) - Student/Researcher/Teacher/Engineer/Other
- area_of_interest (VARCHAR) - predefined option
- experience_level (VARCHAR) - Beginner/Intermediate/Advanced
- topics_of_interest (JSONB) - array of selected topics (nullable)
- completed_at (TIMESTAMP)
- created_at (TIMESTAMP)
- updated_at (TIMESTAMP)

Indexes:
- idx_onboarding_user_id (user_id) - for fast user lookup
- idx_onboarding_completed (completed_at) - for analytics

Constraints:
- Foreign key: user_id references users(id) ON DELETE CASCADE
- Unique: user_id (one profile per user)
```

**Modified Table: `users`**
```
Add column:
- onboarding_completed (BOOLEAN, default FALSE) - quick lookup flag

Migration:
- For existing users, set onboarding_completed = FALSE
- Users created after this feature must complete onboarding
```

**Existing Tables (Unchanged)**:
- `users` - User accounts
- `sessions` - User sessions
- `oauth_accounts` - OAuth provider links
- `conversations` - RAG chatbot conversations (existing)
- `messages` - RAG chatbot messages (existing)

## Assumptions

1. **Better Auth Compatibility**: The existing Better Auth implementation will be compatible with the refactored structure and can be moved without breaking core functionality
2. **Database Access**: Both `rag-chatbot/backend` and `rag-chatbot/auth_backend` will share the same Neon Serverless Postgres database instance with appropriate schema separation
3. **Frontend Routing**: The Docusaurus frontend already has routing logic that can be updated to handle authentication state and onboarding flow without structural changes
4. **Session Storage**: Sessions will use HTTP-only cookies for security, with support for token refresh
5. **Environment Configuration**: All backend modules will share a unified `.env` configuration file for database URLs, OAuth credentials, and CORS origins
6. **OAuth Credentials**: Valid OAuth credentials for Google and GitHub are already configured and will continue to work after refactoring
7. **Development Workflow**: Developers are comfortable with Python/FastAPI and can run the unified backend with `uvicorn` or similar ASGI server
8. **Data Migration**: Existing users (if any) will need to complete onboarding on their next login; this is acceptable for early-stage development
9. **Onboarding Options**: The predefined options for user types, areas of interest, and topics are finalized and won't change during initial implementation
10. **API-First Design**: Frontend is fully decoupled and consumes backend APIs; no server-side rendering or tight coupling exists

## Constraints

1. **No Frontend Restructuring**: The existing Docusaurus frontend structure must remain unchanged; only API integrations and routing logic can be modified
2. **Single Port Requirement**: All backend services MUST run on a single port (e.g., 8000) to avoid CORS and redirect issues
3. **Preserve RAG Functionality**: Existing RAG chatbot backend functionality must remain 100% operational without modifications to core logic
4. **Database**: Must use Neon Serverless Postgres (existing choice)
5. **Better Auth Integration**: Must maintain compatibility with Better Auth framework APIs and patterns
6. **OAuth Providers**: Limited to Google and GitHub OAuth for initial implementation
7. **Onboarding Mandatory**: All new users MUST complete onboarding before accessing the chatbot (no skip option)
8. **Session Management**: Must use secure, production-ready session handling (HTTP-only cookies, CSRF protection if needed)
9. **Backward Compatibility**: If any existing test users exist, they should not lose access (graceful migration to new onboarding requirement)
10. **Development Environment**: Must work in local development (localhost) and be deployable to production without architectural changes

## Non-Goals

1. **Email Verification**: Email verification during registration is out of scope for this feature (can be added later)
2. **Password Reset Flow**: Password reset/forgot password functionality is not included in this iteration
3. **Multi-Factor Authentication (MFA)**: MFA is not required for initial implementation
4. **User Profile Editing**: Ability to edit onboarding data after submission is not included (view-only after completion)
5. **Admin Dashboard**: No admin interface for managing users or viewing onboarding data
6. **Analytics Integration**: No analytics tracking for onboarding completion rates or user behavior
7. **Rate Limiting**: API rate limiting is not included in this specification
8. **Advanced Session Management**: Features like "remember me," session device tracking, or concurrent session limits are out of scope
9. **Social Profile Sync**: Importing additional data from OAuth providers (profile pictures, bio, etc.) beyond basic authentication
10. **Custom User Types**: Ability for users to add custom user types or interests beyond predefined options
11. **Onboarding Localization**: Multi-language support for onboarding forms
12. **Progressive Onboarding**: Breaking onboarding into multiple steps or allowing partial completion

## Dependencies

- **External Dependencies**:
  - Neon Serverless Postgres (database)
  - Google OAuth API (authentication provider)
  - GitHub OAuth API (authentication provider)
  - Better Auth framework (authentication library)

- **Internal Dependencies**:
  - Existing RAG chatbot backend (`rag-chatbot/backend`)
  - Existing Docusaurus frontend (`AIdd-book/`)
  - Existing auth backend code (to be refactored from `rag-chatbot/backend/auth_backend`)

- **Feature Dependencies**:
  - Unified backend (FR-001) must be completed before authentication flows can work
  - Registration/login (FR-005, FR-006, FR-007) must work before onboarding can function
  - Onboarding (FR-013-FR-017) must be complete before protected chatbot access can be enforced

## Open Questions

None - all requirements are clearly defined with reasonable defaults assumed where appropriate.
