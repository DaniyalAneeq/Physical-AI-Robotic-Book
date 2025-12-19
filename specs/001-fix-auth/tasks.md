# Tasks: Authentication Session Persistence Fix

**Input**: Design documents from `/specs/001-fix-auth/`
**Prerequisites**: plan.md (✅ complete), spec.md (✅ complete)

**Tests**: This feature includes comprehensive testing as specified in spec.md Testing Requirements section. Tests are organized by user story.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

This is a web application with:
- **Backend**: `rag-chatbot/auth_backend/` (FastAPI Python)
- **Frontend**: `AIdd-book/src/` (Docusaurus React/TypeScript)
- **E2E Tests**: `qa-automation/tests/` (Playwright)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Environment preparation and configuration validation

- [X] T001 Verify Python 3.11+ and Node.js installed per plan.md technical context
- [X] T002 [P] Install backend dependencies: pip install fastapi starlette psycopg3 pydantic pytest in rag-chatbot/auth_backend/
- [X] T003 [P] Install frontend dependencies: npm install in AIdd-book/ (React 19.0.0, Docusaurus 3.9.2)
- [X] T004 [P] Install E2E test dependencies: pip install playwright pytest-playwright httpx in qa-automation/
- [X] T005 Create .env.development and .env.production template files in rag-chatbot/auth_backend/ with cookie/CORS configuration
- [X] T006 Document environment variables (SECURE_COOKIES, SAME_SITE_COOKIES, CORS_ORIGINS, SESSION_MAX_AGE_DAYS) in deployment checklist

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core cookie/CORS infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Update rag-chatbot/auth_backend/config.py to add environment-specific cookie configuration (SECURE_COOKIES, SAME_SITE_COOKIES from environment variables)
- [X] T008 Update rag-chatbot/auth_backend/config.py to add CORS_ORIGINS parsing from environment variable (comma-separated list)
- [X] T009 Add cookie attribute helper function in rag-chatbot/auth_backend/services/session.py to return correct SameSite/Secure based on environment
- [X] T010 Update CORS middleware in rag-chatbot/auth_backend/main.py to use explicit CORS_ORIGINS list and allow_credentials=True
- [X] T011 [P] Write unit test for cookie configuration in rag-chatbot/auth_backend/tests/test_config.py (dev: Lax/not Secure, prod: None/Secure)
- [X] T012 [P] Write unit test for CORS origins parsing in rag-chatbot/auth_backend/tests/test_config.py
- [X] T013 Run configuration tests to verify they FAIL before implementation (T007-T010)
- [X] T014 Implement T007-T010 to make configuration tests PASS

**Checkpoint**: Foundation ready - cookie and CORS configuration correct for all environments

---

## Phase 3: User Story 1 - Email/Password Authentication Flow (Priority: P1) 🎯 MVP

**Goal**: Users can log in with email/password and immediately access protected routes without re-authentication

**Independent Test**: Register a new account and immediately navigate to `/profile` without logout. Session should persist across page refreshes.

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T015 [P] [US1] Write unit test for /auth/register endpoint setting session cookie with correct attributes in rag-chatbot/auth_backend/tests/test_auth_routes.py
- [ ] T016 [P] [US1] Write unit test for /auth/login endpoint setting session cookie with correct attributes in rag-chatbot/auth_backend/tests/test_auth_routes.py
- [ ] T017 [P] [US1] Write unit test for /auth/logout endpoint clearing session cookie in rag-chatbot/auth_backend/tests/test_auth_routes.py
- [ ] T018 [P] [US1] Write unit test for /auth/session endpoint returning 401 when no cookie present in rag-chatbot/auth_backend/tests/test_auth_routes.py
- [ ] T019 [P] [US1] Write unit test for /auth/session endpoint returning user data when valid cookie present in rag-chatbot/auth_backend/tests/test_auth_routes.py
- [ ] T020 [P] [US1] Write integration test for email/password registration → session cookie → profile access in rag-chatbot/auth_backend/tests/test_integration_auth.py
- [ ] T021 [P] [US1] Write E2E test for complete registration journey with onboarding in qa-automation/tests/test_auth_flow.py
- [ ] T022 [US1] Run User Story 1 tests to verify they FAIL before implementation

### Implementation for User Story 1

- [X] T023 [P] [US1] Update /auth/register endpoint in rag-chatbot/auth_backend/api/routes/auth.py to use cookie attribute helper from T009
- [X] T024 [P] [US1] Update /auth/login endpoint in rag-chatbot/auth_backend/api/routes/auth.py to use cookie attribute helper from T009
- [X] T025 [P] [US1] Update /auth/logout endpoint in rag-chatbot/auth_backend/api/routes/auth.py to clear cookie with matching domain/path
- [X] T026 [US1] Verify /auth/session endpoint validates session cookie and returns 401 on invalid/missing cookie in rag-chatbot/auth_backend/api/routes/auth.py
- [X] T027 [US1] Add logging for authentication failures (invalid tokens, expired sessions) in rag-chatbot/auth_backend/api/routes/auth.py per FR-034
- [ ] T028 [US1] Run User Story 1 tests to verify they PASS after implementation
- [ ] T029 [US1] Manual test: Register → verify DevTools shows correct cookie attributes (SameSite, Secure, HttpOnly, Path) → navigate to /profile → verify authenticated

**Checkpoint**: Email/password authentication fully functional with persistent sessions

---

## Phase 4: User Story 4 - Cross-Origin Session Persistence (Priority: P1)

**Goal**: Application maintains session cookies correctly across different origins (localhost dev, GitHub Pages prod) and different ports

**Independent Test**: Log in on localhost:3000 (frontend) and verify cookie is accessible when making requests to localhost:8000 (backend). DevTools should show cookie attributes appropriate for environment.

### Tests for User Story 4

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T030 [P] [US4] Write integration test for cross-origin cookie transmission (port 3000 → port 8000) in rag-chatbot/auth_backend/tests/test_integration_cors.py
- [ ] T031 [P] [US4] Write unit test verifying CORS response headers include Access-Control-Allow-Credentials: true in rag-chatbot/auth_backend/tests/test_cors_middleware.py
- [ ] T032 [P] [US4] Write unit test verifying CORS origins list excludes wildcard (*) when credentials enabled in rag-chatbot/auth_backend/tests/test_cors_middleware.py
- [ ] T033 [P] [US4] Write frontend unit test verifying all auth API calls include credentials: "include" in AIdd-book/src/__tests__/services/authApi.test.ts
- [ ] T034 [US4] Run User Story 4 tests to verify they FAIL before implementation

### Implementation for User Story 4

- [X] T035 [US4] Verify CORS middleware implementation from T010 allows credentials and uses explicit origins (should already be complete)
- [X] T036 [US4] Verify all fetch calls in AIdd-book/src/services/authApi.ts include credentials: "include" (should already be present per spec analysis)
- [ ] T037 [US4] Run User Story 4 tests to verify they PASS after implementation
- [ ] T038 [US4] Manual test: Start frontend on port 3000 and backend on port 8000 → login → verify DevTools Network tab shows cookie in request headers → verify CORS headers in response

**Checkpoint**: Cross-origin authentication works in both dev (HTTP, different ports) and prod (HTTPS, different domains)

---

## Phase 5: User Story 2 - Google OAuth Authentication Flow (Priority: P1)

**Goal**: Users can authenticate via Google OAuth and remain authenticated after callback redirect with onboarding presented if needed

**Independent Test**: Click "Login with Google", complete OAuth consent, and verify persistent authentication after redirect. New users should see onboarding modal.

### Tests for User Story 2

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T039 [P] [US2] Write unit test for /auth/oauth/google/callback creating session and redirecting with token in fragment in rag-chatbot/auth_backend/tests/test_oauth_routes.py
- [ ] T040 [P] [US2] Write unit test for /auth/session-from-token validating token and setting session cookie in rag-chatbot/auth_backend/tests/test_oauth_routes.py
- [ ] T041 [P] [US2] Write unit test for OAuth callback including onboarding_required flag in redirect URL in rag-chatbot/auth_backend/tests/test_oauth_routes.py
- [ ] T042 [P] [US2] Write unit test for OAuth error handling redirecting to login with error message in rag-chatbot/auth_backend/tests/test_oauth_routes.py
- [ ] T043 [P] [US2] Write frontend unit test for token extraction from URL fragment in AIdd-book/src/__tests__/pages/auth/callback.test.tsx
- [ ] T044 [P] [US2] Write frontend unit test for /session-from-token exchange before redirect in AIdd-book/src/__tests__/pages/auth/callback.test.tsx
- [ ] T045 [P] [US2] Write integration test for Google OAuth flow → callback exchange → verify session cookie in rag-chatbot/auth_backend/tests/test_integration_oauth.py
- [ ] T046 [P] [US2] Write E2E test for Google OAuth journey with mocked consent in qa-automation/tests/test_oauth_flow.py
- [ ] T047 [US2] Run User Story 2 tests to verify they FAIL before implementation

### Implementation for User Story 2

- [X] T048 [P] [US2] Update OAuth callback in rag-chatbot/auth_backend/api/routes/oauth.py to use cookie attribute helper from T009 for session creation (cookies set by /session-from-token endpoint, not callbacks)
- [X] T049 [P] [US2] Update /session-from-token endpoint in rag-chatbot/auth_backend/api/routes/oauth.py to use cookie attribute helper from T009 (auth.py:443)
- [X] T050 [US2] Add session verification step in AIdd-book/src/pages/auth/callback.tsx after /session-from-token exchange before redirect (FR-019) (callback.tsx:62-76)
- [X] T051 [US2] Update callback handler in AIdd-book/src/pages/auth/callback.tsx to call /auth/session to verify session established before final redirect (callback.tsx:62-76)
- [X] T052 [US2] Add OAuth error handling in AIdd-book/src/pages/auth/callback.tsx to redirect to login with error message on invalid/expired token (callback.tsx:75-84)
- [ ] T053 [US2] Run User Story 2 tests to verify they PASS after implementation
- [ ] T054 [US2] Manual test: Login with Google → complete consent (or use test account) → verify callback exchange → verify session cookie → verify authenticated

**Checkpoint**: Google OAuth authentication fully functional with session persistence and error handling

---

## Phase 6: User Story 3 - Onboarding Completion After Signup (Priority: P2)

**Goal**: New users are immediately presented with an onboarding modal to personalize their experience after registration

**Independent Test**: Register a new account and verify the onboarding modal appears immediately. Complete onboarding and verify it doesn't reappear on next login.

### Tests for User Story 3

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T055 [P] [US3] Write frontend unit test for automatic onboarding modal trigger when onboarding_required: true in AIdd-book/src/__tests__/hooks/useAuth.test.tsx
- [ ] T056 [P] [US3] Write frontend unit test for onboarding modal persistence until completion in AIdd-book/src/__tests__/hooks/useAuth.test.tsx
- [ ] T057 [P] [US3] Write integration test for signup → onboarding modal → complete onboarding → verify onboarding_completed: true in rag-chatbot/auth_backend/tests/test_integration_onboarding.py
- [ ] T058 [P] [US3] Write E2E test for onboarding flow (register → modal appears → complete → doesn't reappear) in qa-automation/tests/test_onboarding_flow.py
- [ ] T059 [US3] Run User Story 3 tests to verify they FAIL before implementation

### Implementation for User Story 3

- [X] T060 [US3] Add automatic onboarding modal trigger logic in AIdd-book/src/hooks/useAuth.tsx register() function when onboarding_required: true (useAuth.tsx:136-138)
- [X] T061 [US3] Add automatic onboarding modal trigger logic in AIdd-book/src/hooks/useAuth.tsx loadSession() function when onboarding_required: true (useAuth.tsx:61-63)
- [X] T062 [US3] Update auth response logic in rag-chatbot/auth_backend/api/routes/auth.py to check onboarding_completed flag and set onboarding_required accordingly (FR-025) (auth.py:107,192,341,380,455)
- [X] T063 [US3] Ensure onboarding modal reappears on page reload if onboarding not completed (FR-023) in AIdd-book/src/hooks/useAuth.tsx (loadSession called on mount, line 88)
- [ ] T064 [US3] Run User Story 3 tests to verify they PASS after implementation
- [ ] T065 [US3] Manual test: Register new account → verify modal appears within 2 seconds → complete onboarding → logout → login → verify modal does NOT reappear

**Checkpoint**: Onboarding flow fully functional with automatic triggering and persistence

---

## Phase 7: User Story 5 - Session Recovery After Page Reload (Priority: P2)

**Goal**: Users refresh the page or return to the app, and their authentication state is restored automatically

**Independent Test**: Log in, refresh the page, and verify user remains authenticated without re-login. Open app in new tab and verify session is recognized.

### Tests for User Story 5

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T066 [P] [US5] Write frontend unit test for loadSession() updating user state on success in AIdd-book/src/__tests__/hooks/useAuth.test.tsx
- [ ] T067 [P] [US5] Write frontend unit test for loadSession() clearing user state on 401 in AIdd-book/src/__tests__/hooks/useAuth.test.tsx
- [ ] T068 [P] [US5] Write unit test for backend session validation rejecting expired sessions with 401 in rag-chatbot/auth_backend/tests/test_session_service.py
- [ ] T069 [P] [US5] Write integration test for login → simulate page refresh → verify session restored from cookie in rag-chatbot/auth_backend/tests/test_integration_session_persistence.py
- [ ] T070 [P] [US5] Write E2E test for session persistence across page refreshes and new tabs in qa-automation/tests/test_session_persistence.py
- [ ] T071 [US5] Run User Story 5 tests to verify they FAIL before implementation

### Implementation for User Story 5

- [X] T072 [US5] Verify loadSession() implementation in AIdd-book/src/hooks/useAuth.tsx queries /auth/session on mount (should already be present) (useAuth.tsx:56, called by useEffect line 88)
- [X] T073 [US5] Verify session expiry validation in rag-chatbot/auth_backend/services/session.py checks expires_at timestamp (FR-032) (session.py:92-93 in is_valid())
- [X] T074 [US5] Verify frontend handles 401 responses by clearing user state and redirecting to login in AIdd-book/src/hooks/useAuth.tsx (FR-027) (useAuth.tsx:79-81 sets user to null on error)
- [X] T075 [US5] Add refreshSession() method to auth context in AIdd-book/src/hooks/useAuth.tsx to manually revalidate session (FR-029) (useAuth.tsx:176-178)
- [ ] T076 [US5] Run User Story 5 tests to verify they PASS after implementation
- [ ] T077 [US5] Manual test: Login → refresh page → verify still authenticated → open new tab → verify session recognized → wait for session expiry (or manually expire in DB) → refresh → verify logged out

**Checkpoint**: Session recovery and expiry handling fully functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, validation, and deployment readiness

- [X] T078 [P] Create deployment checklist in specs/001-fix-auth/deployment-checklist.md documenting cookie/CORS configuration per FR-039 (DEPLOYMENT.md exists with comprehensive checklist)
- [X] T079 [P] Add cookie/CORS debugging guide to specs/001-fix-auth/debugging.md (already embedded in plan.md quickstart guide - extract to standalone file) (debugging.md created)
- [X] T080 [P] Update project README with authentication configuration instructions (auth_backend/README.md updated with correct cookie config)
- [ ] T081 Run full E2E test suite (all tests from User Stories 1-5) to verify end-to-end functionality
- [ ] T082 Run manual verification using specs/001-fix-auth/plan.md Phase 1 quickstart verification checklist
- [X] T083 [P] Code cleanup: Remove any debug logging added during development (no debug logging added - only required feature logging per FR-034)
- [X] T084 [P] Security review: Verify no secrets in frontend code, verify HttpOnly cookies, verify CORS origins list (verified: no secrets, httponly=True, explicit origins list)
- [ ] T085 Performance test: Verify session validation p95 latency <100ms, OAuth callback flow completes within 3 seconds
- [X] T086 Document known limitations and edge cases in specs/001-fix-auth/known-issues.md (session expiry during active use, concurrent logins, cookie blocking) (known-issues.md created with 22 documented limitations)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Can start after Phase 2 complete
- **User Story 4 (Phase 4)**: Depends on Foundational - Can start after Phase 2 complete (parallel with US1)
- **User Story 2 (Phase 5)**: Depends on Foundational - Can start after Phase 2 complete (parallel with US1, US4)
- **User Story 3 (Phase 6)**: Depends on User Story 1 (needs auth context) - Can start after US1 complete
- **User Story 5 (Phase 7)**: Depends on User Story 1 (needs session management) - Can start after US1 complete (parallel with US3)
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies on other stories - Can start after Foundational (Phase 2)
- **User Story 4 (P1)**: No dependencies on other stories - Can start after Foundational (Phase 2) - PARALLEL with US1
- **User Story 2 (P1)**: No dependencies on other stories - Can start after Foundational (Phase 2) - PARALLEL with US1, US4
- **User Story 3 (P2)**: Depends on User Story 1 (needs auth context register() function) - Can start after US1
- **User Story 5 (P2)**: Depends on User Story 1 (needs session validation) - Can start after US1 - PARALLEL with US3

### Within Each User Story

- Tests MUST be written and FAIL before implementation (T022, T034, T047, T059, T071)
- Implementation tasks can run in parallel if marked [P] and different files
- Verify tests PASS after implementation before moving to next story

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T002, T003, T004 can run in parallel (different directories)

**Foundational Phase (Phase 2)**:
- T011, T012 can run in parallel (different test files)

**After Foundational Complete, These User Stories Can Start in Parallel**:
- User Story 1 (T015-T029)
- User Story 4 (T030-T038)
- User Story 2 (T039-T054)

**User Story 1 Tests (all can run in parallel)**:
- T015, T016, T017, T018, T019, T020, T021 (different test files or different test functions)

**User Story 1 Implementation (parallel opportunities)**:
- T023, T024, T025 (different functions in same file, minimal conflict)

**User Story 4 Tests (all can run in parallel)**:
- T030, T031, T032, T033 (different test files)

**User Story 2 Tests (all can run in parallel)**:
- T039, T040, T041, T042, T043, T044, T045, T046 (different test files)

**User Story 2 Implementation (parallel opportunities)**:
- T048, T049 (backend routes), T050, T051, T052 (frontend callback) can run in parallel

**After User Story 1 Complete, These Stories Can Start in Parallel**:
- User Story 3 (T055-T065)
- User Story 5 (T066-T077)

**Polish Phase (parallel opportunities)**:
- T078, T079, T080, T083, T084 can run in parallel (different files, different concerns)

---

## Parallel Example: After Foundational Phase

```bash
# Three developers can work in parallel after Phase 2 completes:

# Developer A: User Story 1 (Email/Password Auth)
Tasks: T015-T029 (tests → implementation → verification)

# Developer B: User Story 4 (Cross-Origin Persistence)
Tasks: T030-T038 (tests → implementation → verification)

# Developer C: User Story 2 (Google OAuth)
Tasks: T039-T054 (tests → implementation → verification)

# Once US1 completes, Developer A can start:
Tasks: T055-T065 (User Story 3: Onboarding)

# Developer B can start after US1:
Tasks: T066-T077 (User Story 5: Session Recovery)
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 4 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T014) - CRITICAL, blocks all stories
3. Complete Phase 3: User Story 1 (T015-T029) - Email/password auth
4. Complete Phase 4: User Story 4 (T030-T038) - Cross-origin persistence
5. **STOP and VALIDATE**: Test email/password login works in dev and prod environments
6. Deploy/demo if ready (MVP: basic auth working)

### Incremental Delivery

1. Setup + Foundational (T001-T014) → Foundation ready
2. Add User Story 1 (T015-T029) → Test independently → Email/password auth works
3. Add User Story 4 (T030-T038) → Test independently → Cross-origin works in all environments
4. Add User Story 2 (T039-T054) → Test independently → OAuth auth works (MVP++)
5. Add User Story 3 (T055-T065) → Test independently → Onboarding experience complete
6. Add User Story 5 (T066-T077) → Test independently → Session persistence complete
7. Polish (T078-T086) → Production ready

### Parallel Team Strategy

With 3 developers:

1. All devs complete Setup (T001-T006) together
2. All devs complete Foundational (T007-T014) together
3. Once Foundational done:
   - Dev A: User Story 1 (T015-T029)
   - Dev B: User Story 4 (T030-T038)
   - Dev C: User Story 2 (T039-T054)
4. Once US1 done:
   - Dev A: User Story 3 (T055-T065)
   - Dev B: User Story 5 (T066-T077)
   - Dev C: Continues US2 or helps with US3/US5
5. All devs: Polish (T078-T086) together

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label (US1-US5) maps task to specific user story for traceability
- Each user story should be independently testable and deliverable
- **TDD Workflow**: Write tests → Run tests (expect FAIL) → Implement → Run tests (expect PASS)
- Commit after each checkpoint or logical task group
- Stop at any checkpoint to validate story independently
- **Testing Strategy**: Unit tests (backend/frontend) → Integration tests → E2E tests
- **Environment Testing**: Test in both dev (HTTP, localhost:3000→8000) and prod (HTTPS, GitHub Pages→Render) configurations
- Avoid: vague tasks, same-file conflicts, cross-story dependencies that break independence

---

## Summary Statistics

- **Total Tasks**: 86
- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 8 tasks (BLOCKS all user stories)
- **Phase 3 (User Story 1)**: 15 tasks (7 tests, 7 implementation, 1 manual verification)
- **Phase 4 (User Story 4)**: 9 tasks (5 tests, 4 implementation/verification)
- **Phase 5 (User Story 2)**: 16 tasks (9 tests, 6 implementation, 1 manual verification)
- **Phase 6 (User Story 3)**: 11 tasks (5 tests, 5 implementation, 1 manual verification)
- **Phase 7 (User Story 5)**: 12 tasks (6 tests, 5 implementation, 1 manual verification)
- **Phase 8 (Polish)**: 9 tasks (documentation, validation, deployment readiness)

**Parallel Opportunities Identified**: 41 tasks marked [P] can run in parallel within their phase

**MVP Scope**: Phases 1-4 (39 tasks) deliver email/password authentication with cross-origin persistence

**Full Feature Scope**: All 86 tasks deliver complete authentication fix with OAuth, onboarding, and session recovery
