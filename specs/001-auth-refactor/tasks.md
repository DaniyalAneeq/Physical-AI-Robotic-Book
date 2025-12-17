# Tasks: Authentication Refactor & Integration

**Input**: Design documents from `/specs/001-auth-refactor/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Contract and integration tests included as specified in FR-023 (Automated Testing)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `rag-chatbot/backend/` (RAG + unified server), `rag-chatbot/auth_backend/` (auth module), `AIdd-book/` (frontend)
- Backend paths: `rag-chatbot/backend/app/` for RAG, `rag-chatbot/auth_backend/` for auth
- Frontend paths: `AIdd-book/src/` for Docusaurus components

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory refactoring

- [ ] T001 [P1] [US1] Move `rag-chatbot/backend/auth_backend` to `rag-chatbot/auth_backend` (top-level directory refactor)
- [ ] T002 [P1] [US1] Update all import statements in `rag-chatbot/auth_backend/` to reflect new module path
- [ ] T003 [P1] [US1] Create `.env.example` file in `rag-chatbot/backend/` with OAuth and session configuration template
- [ ] T004 [P] Configure pytest with coverage in `rag-chatbot/backend/pyproject.toml`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Foundation

- [ ] T005 Create database migration `rag-chatbot/auth_backend/migrations/002_onboarding.py` (add `onboarding_completed` flag, create `onboarding_profiles` table)
- [ ] T006 [P] Create SQLAlchemy model for OnboardingProfile in `rag-chatbot/auth_backend/models/onboarding_profile.py`
- [ ] T007 [P] Update User model in `rag-chatbot/auth_backend/models/user.py` to include `onboarding_completed` field

### Application Foundation

- [ ] T008 Create unified FastAPI app in `rag-chatbot/backend/app/main.py` integrating auth and RAG routers
- [ ] T009 Configure CORS middleware in `rag-chatbot/backend/app/main.py` for `http://localhost:3000` origin
- [ ] T010 [P] Update backend config in `rag-chatbot/backend/app/config.py` to include OAuth and session settings 
- [ ] T011 [P] Create authentication dependencies in `rag-chatbot/auth_backend/api/deps.py` (`get_current_user`, `get_current_user_with_onboarding`)

### Pydantic Schemas

- [ ] T012 [P] Create onboarding schemas in `rag-chatbot/auth_backend/schemas/onboarding.py` (OnboardingRequest, OnboardingStatusResponse, OnboardingProfileResponse, OnboardingOptionsResponse)
- [ ] T013 [P] Update auth schemas in `rag-chatbot/auth_backend/schemas/auth.py` to include `onboarding_required` field in AuthResponse

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Unified Backend Architecture (Priority: P1) 🎯 MVP

**Goal**: Single FastAPI server on port 8000 serving both RAG (`/api/*`) and auth (`/auth/*`) endpoints

**Independent Test**: Start server with `uvicorn app.main:app --port 8000`, verify `/api/health` and `/auth/*` routes accessible via Swagger UI at `/docs`

### Implementation for User Story 1

- [ ] T014 [US1] Register RAG routers in `rag-chatbot/backend/app/main.py` with `/api` prefix (chatkit, conversations, health, sessions)
- [ ] T015 [US1] Register auth routers in `rag-chatbot/backend/app/main.py` with `/auth` prefix (auth, oauth, onboarding)
- [ ] T016 [US1] Update `rag-chatbot/backend/app/api/health.py` to verify both RAG and auth modules are loaded
- [ ] T017 [US1] Remove or deprecate old `run.py` or separate auth server startup scripts

### Tests for User Story 1

- [ ] T018 [P] [US1] Contract test for unified server startup in `rag-chatbot/backend/tests/contract/test_unified_server.py` (verify all routes registered)
- [ ] T019 [P] [US1] Integration test for CORS configuration in `rag-chatbot/backend/tests/integration/test_cors.py`

**Checkpoint**: At this point, User Story 1 should be fully functional - single server running on port 8000 with all routes

---

## Phase 4: User Story 2 - User Registration and Login (Priority: P2)

**Goal**: Users can register with email/password or OAuth (Google/GitHub), receive session cookies, auto-login after registration

**Independent Test**: POST `/auth/register` with valid data → 201 response + session cookie, POST `/auth/login` → 200 response + session cookie, GET `/auth/me` with cookie → user data

### OAuth Configuration

- [ ] T020 [P] [US2] Create OAuth service in `rag-chatbot/auth_backend/services/oauth.py` (Google and GitHub provider initialization using Authlib)
- [ ] T021 [P] [US2] Implement OAuth routes in `rag-chatbot/auth_backend/api/routes/oauth.py` (`/auth/oauth/google`, `/auth/oauth/google/callback`, `/auth/oauth/github`, `/auth/oauth/github/callback`)

### Session Management

- [ ] T022 [US2] Update session service in `rag-chatbot/auth_backend/services/session.py` to set HTTP-only cookies with secure, SameSite=strict flags
- [ ] T023 [US2] Implement session validation in `rag-chatbot/auth_backend/services/session.py` with sliding window (update `last_used_at` on every request)

### Authentication Routes

- [ ] T024 [US2] Implement registration endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/register`) with auto-login
- [ ] T025 [US2] Implement login endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/login`)
- [ ] T026 [US2] Implement logout endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/logout`) with session cleanup
- [ ] T027 [US2] Implement current user endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`GET /auth/me`)
- [ ] T028 [US2] Implement token refresh endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/refresh`)

### Validation and Error Handling

- [ ] T029 [US2] Add email format validation and password strength validation (min 8 characters) in `rag-chatbot/auth_backend/schemas/auth.py`
- [ ] T030 [US2] Add duplicate email check in registration flow (return 409 Conflict if email exists)
- [ ] T031 [US2] Add invalid credentials error handling in login flow (return 401 Unauthorized)

### Tests for User Story 2

- [ ] T032 [P] [US2] Contract test for `/auth/register` in `rag-chatbot/auth_backend/tests/contract/test_auth_api.py`
- [ ] T033 [P] [US2] Contract test for `/auth/login` in `rag-chatbot/auth_backend/tests/contract/test_auth_api.py`
- [ ] T034 [P] [US2] Contract test for OAuth endpoints in `rag-chatbot/auth_backend/tests/contract/test_oauth_api.py`
- [ ] T035 [P] [US2] Integration test for registration → auto-login flow in `rag-chatbot/auth_backend/tests/integration/test_auth_flow.py`
- [ ] T036 [P] [US2] Integration test for OAuth flow (Google) in `rag-chatbot/auth_backend/tests/integration/test_oauth_flow.py`
- [ ] T037 [P] [US2] Unit test for session cookie configuration in `rag-chatbot/auth_backend/tests/unit/test_session.py`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - users can register, login, and access `/auth/me`

---

## Phase 5: User Story 3 - Mandatory Onboarding (Priority: P2)

**Goal**: After registration/login, users MUST complete onboarding (user type, interests, experience) before accessing chatbot

**Independent Test**: Register user → GET `/auth/onboarding/status` shows `completed: false` → POST `/auth/onboarding/complete` with valid data → status shows `completed: true`

### Onboarding Service

- [ ] T038 [US3] Create onboarding service in `rag-chatbot/auth_backend/services/onboarding.py` (create profile, validate enums, update user flag)
- [ ] T039 [US3] Define predefined options constants in `rag-chatbot/auth_backend/services/onboarding.py` (USER_TYPES, AREAS_OF_INTEREST, EXPERIENCE_LEVELS, TOPICS_OF_INTEREST)

### Onboarding Routes

- [ ] T040 [P] [US3] Implement onboarding status endpoint in `rag-chatbot/auth_backend/api/routes/onboarding.py` (`GET /auth/onboarding/status`)
- [ ] T041 [P] [US3] Implement onboarding options endpoint in `rag-chatbot/auth_backend/api/routes/onboarding.py` (`GET /auth/onboarding/options`)
- [ ] T042 [US3] Implement onboarding completion endpoint in `rag-chatbot/auth_backend/api/routes/onboarding.py` (`POST /auth/onboarding/complete`)
- [ ] T043 [P] [US3] Implement onboarding profile retrieval endpoint in `rag-chatbot/auth_backend/api/routes/onboarding.py` (`GET /auth/onboarding/profile`)

### Validation

- [ ] T044 [US3] Add ENUM validation for user_type, experience_level in `rag-chatbot/auth_backend/schemas/onboarding.py`
- [ ] T045 [US3] Add area_of_interest validation (max 100 characters) in `rag-chatbot/auth_backend/schemas/onboarding.py`
- [ ] T046 [US3] Add topics_of_interest validation (max 10 items, optional) in `rag-chatbot/auth_backend/schemas/onboarding.py`
- [ ] T047 [US3] Add 409 Conflict error if user tries to complete onboarding twice

### Frontend Integration (Minimal - API integration only)

- [ ] T048 [P] [US3] Create onboarding page in `AIdd-book/src/pages/onboarding.tsx` with form fields matching API schema
- [ ] T049 [US3] Add onboarding API calls in `AIdd-book/src/services/authApi.ts` (`getOnboardingStatus`, `getOnboardingOptions`, `completeOnboarding`, `getOnboardingProfile`)
- [ ] T050 [US3] Update auth hook in `AIdd-book/src/hooks/useAuth.tsx` to check onboarding status and redirect to `/onboarding` if not completed

### Tests for User Story 3

- [ ] T051 [P] [US3] Contract test for `/auth/onboarding/status` in `rag-chatbot/auth_backend/tests/contract/test_onboarding_api.py`
- [ ] T052 [P] [US3] Contract test for `/auth/onboarding/complete` in `rag-chatbot/auth_backend/tests/contract/test_onboarding_api.py`
- [ ] T053 [P] [US3] Integration test for full onboarding flow in `rag-chatbot/auth_backend/tests/integration/test_onboarding_flow.py`
- [ ] T054 [P] [US3] Unit test for onboarding profile creation in `rag-chatbot/auth_backend/tests/unit/test_onboarding.py`

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently - users must complete onboarding after authentication

---

## Phase 6: User Story 4 - Protected Chatbot Access (Priority: P3)

**Goal**: RAG chatbot endpoints require authentication + completed onboarding, unauthenticated users redirected to login

**Independent Test**: Access `/api/chat` without auth → 401 Unauthorized, access with auth but no onboarding → 403 Forbidden with "Onboarding required", access with auth + onboarding → 200 OK

### Route Protection

- [ ] T055 [US4] Add `get_current_user_with_onboarding` dependency to `POST /api/chat` in `rag-chatbot/backend/app/api/chatkit.py`
- [ ] T056 [US4] Add `get_current_user_with_onboarding` dependency to `GET /api/sessions` in `rag-chatbot/backend/app/api/sessions.py`
- [ ] T057 [US4] Add `get_current_user_with_onboarding` dependency to `POST /api/sessions` in `rag-chatbot/backend/app/api/sessions.py`
- [ ] T058 [US4] Add `get_current_user_with_onboarding` dependency to `DELETE /api/sessions/{id}` in `rag-chatbot/backend/app/api/sessions.py`
- [ ] T059 [US4] Add `get_current_user_with_onboarding` dependency to `GET /api/conversations` in `rag-chatbot/backend/app/api/conversations.py`

### Error Handling

- [ ] T060 [US4] Implement 401 Unauthorized error response for missing/invalid session tokens
- [ ] T061 [US4] Implement 403 Forbidden error response for authenticated users without completed onboarding (include `Location: /onboarding` header hint)

### Frontend Updates

- [ ] T062 [P] [US4] Update chat page in `AIdd-book/src/pages/chat.tsx` to check authentication status before rendering
- [ ] T063 [US4] Add redirect logic in `AIdd-book/src/theme/Root.tsx` to send unauthenticated users to `/login`
- [ ] T064 [US4] Add redirect logic in `AIdd-book/src/hooks/useAuth.tsx` to send authenticated but non-onboarded users to `/onboarding`

### Tests for User Story 4

- [ ] T065 [P] [US4] Contract test for protected endpoint authentication in `rag-chatbot/backend/tests/contract/test_protected_routes.py`
- [ ] T066 [P] [US4] Integration test for unauthenticated access → 401 in `rag-chatbot/backend/tests/integration/test_auth_enforcement.py`
- [ ] T067 [P] [US4] Integration test for authenticated without onboarding → 403 in `rag-chatbot/backend/tests/integration/test_onboarding_enforcement.py`
- [ ] T068 [P] [US4] Integration test for full auth + onboarding → chatbot access in `rag-chatbot/backend/tests/integration/test_full_access_flow.py`

**Checkpoint**: At this point, all core user stories (1-4) should work independently - chatbot is fully protected

---

## Phase 7: User Story 5 - Session Management (Priority: P4)

**Goal**: Sessions expire after 30 days absolute, 7 days idle, users can manually logout, tokens can be refreshed

**Independent Test**: Login → wait for session expiration → GET `/auth/me` returns 401, POST `/auth/refresh` with valid session → new session cookie

### Session Expiration Logic

- [ ] T069 [US5] Implement absolute expiration check (30 days) in `rag-chatbot/auth_backend/services/session.py` (`validate_session`)
- [ ] T070 [US5] Implement idle timeout check (7 days since last_used_at) in `rag-chatbot/auth_backend/services/session.py` (`validate_session`)
- [ ] T071 [US5] Update `last_used_at` timestamp on every authenticated request in `rag-chatbot/auth_backend/api/deps.py` (`get_current_user`)

### Token Refresh

- [ ] T072 [US5] Implement token refresh logic in `rag-chatbot/auth_backend/services/session.py` (`refresh_session` - create new session, invalidate old token)
- [ ] T073 [US5] Ensure refresh endpoint in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/refresh`) uses refresh logic

### Session Cleanup

- [ ] T074 [US5] Create database cleanup task to delete expired sessions from `sessions` table (cron job or background task)
- [ ] T075 [US5] Implement logout cleanup in `rag-chatbot/auth_backend/api/routes/auth.py` (`POST /auth/logout`) to delete session record and clear cookie

### Tests for User Story 5

- [ ] T076 [P] [US5] Unit test for absolute expiration (30 days) in `rag-chatbot/auth_backend/tests/unit/test_session_expiration.py`
- [ ] T077 [P] [US5] Unit test for idle timeout (7 days) in `rag-chatbot/auth_backend/tests/unit/test_session_expiration.py`
- [ ] T078 [P] [US5] Integration test for session refresh flow in `rag-chatbot/auth_backend/tests/integration/test_session_refresh.py`
- [ ] T079 [P] [US5] Integration test for logout cleanup in `rag-chatbot/auth_backend/tests/integration/test_logout.py`

**Checkpoint**: All user stories (1-5) should now be independently functional - full session lifecycle implemented

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Documentation

- [ ] T080 [P] Update main README in `rag-chatbot/README.md` with unified server architecture documentation
- [ ] T081 [P] Update OAuth setup instructions in `specs/001-auth-refactor/quickstart.md` based on actual implementation
- [ ] T082 [P] Create API documentation in `rag-chatbot/backend/docs/API.md` combining auth and RAG endpoint documentation

### Security Hardening

- [ ] T083 Add rate limiting for authentication endpoints (login, register) using slowapi or custom middleware
- [ ] T084 Implement OAuth token encryption for `access_token` and `refresh_token` fields in `oauth_accounts` table
- [ ] T085 Add security headers middleware (HSTS, X-Content-Type-Options, X-Frame-Options)

### Error Handling and Logging

- [ ] T086 [P] Implement structured logging for authentication events (login success/failure, registration, logout) in `rag-chatbot/auth_backend/services/logging.py`
- [ ] T087 [P] Ensure generic error messages to clients, detailed logs server-side (no password or token leaks)

### Performance

- [ ] T088 Add database connection pooling configuration in `rag-chatbot/backend/app/config.py`
- [ ] T089 Add index on `sessions.last_used_at` for idle timeout query performance

### Quickstart Validation

- [ ] T090 Run through `specs/001-auth-refactor/quickstart.md` end-to-end to validate setup instructions
- [ ] T091 Test OAuth flow with Google OAuth provider using real credentials
- [ ] T092 Test OAuth flow with GitHub OAuth provider using real credentials

### Deployment Preparation

- [ ] T093 Create production `.env.example` file with placeholder values for all required environment variables
- [ ] T094 Update `rag-chatbot/backend/Dockerfile` (if exists) to use unified server entry point
- [ ] T095 Create deployment checklist in `specs/001-auth-refactor/deployment.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories (independent)
- **User Story 3 (P2)**: Depends on User Story 2 (requires authentication to work)
- **User Story 4 (P3)**: Depends on User Story 2 and User Story 3 (requires auth + onboarding)
- **User Story 5 (P4)**: Depends on User Story 2 (enhances session management from US2)

### Within Each User Story

- Tests (contract/integration) can be written first and should FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T001, T002, T003 are independent)
- All Foundational tasks marked [P] can run in parallel (T006, T007, T010, T011, T012, T013)
- User Story 1 and User Story 2 can be worked on in parallel (independent)
- Tests within a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Polish tasks marked [P] can run in parallel

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Unified Backend)
4. Complete Phase 4: User Story 2 (Registration/Login)
5. **STOP and VALIDATE**: Test authentication flow independently
6. Deploy/demo if ready (basic auth working without onboarding enforcement)

### Full Feature Delivery (All User Stories)

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Unified server running
3. Add User Story 2 → Test independently → Auth working
4. Add User Story 3 → Test independently → Onboarding mandatory
5. Add User Story 4 → Test independently → Chatbot protected
6. Add User Story 5 → Test independently → Session management complete
7. Polish phase → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Unified Backend)
   - Developer B: User Story 2 (Auth)
   - Developer C: User Story 3 (Onboarding) - starts after US2 complete
3. Developer A picks up User Story 4 after US1 complete
4. Developer B picks up User Story 5 after US2 complete
5. Team collaborates on Polish phase

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (red → green → refactor)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- **Critical**: Do NOT modify existing RAG backend logic - only add authentication dependencies
- **Critical**: All OAuth redirect URLs must exactly match single-port architecture (localhost:8000 in dev)
- **Critical**: Session cookies MUST be HTTP-only, Secure (production), SameSite=strict
