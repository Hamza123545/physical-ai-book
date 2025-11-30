# Tasks: Better Auth Integration

**Input**: Design documents from `/specs/better-auth/`
**Prerequisites**: plan.md (created), spec.md (requirements documented in plan)

**Tests**: Not explicitly requested - focused on implementation and integration

**Organization**: Tasks are grouped by functional capability (user story) to enable independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (FastAPI structure)
- **Frontend**: `book-source/src/components/Auth/`
- Paths follow web app structure as defined in plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and auth infrastructure setup

- [ ] T001 Create auth-related directory structure: backend/app/middleware/, backend/app/models/{user,user_profile,session}.py
- [ ] T002 Add python-jose[cryptography], passlib[bcrypt], python-multipart to backend/requirements.txt
- [ ] T003 [P] Add JWT_SECRET, JWT_ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES to backend/.env.example
- [ ] T004 [P] Create JWT configuration in backend/app/config.py (secret, algorithm, expiration)
- [ ] T005 [P] Install Better Auth React SDK: add better-auth and jwt-decode to book-source/package.json

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core auth infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create User model: users table with id, email, email_verified, name, image, timestamps in backend/app/models/user.py
- [ ] T007 Create Session model: sessions table with id, user_id FK, expires_at, session_token, created_at in backend/app/models/session.py
- [ ] T008 Create UserProfile model: user_profiles table with user_id FK, software_experience, hardware_experience, learning_goals, preferred_topics JSONB in backend/app/models/user_profile.py
- [ ] T009 [P] Create Pydantic schemas for auth requests/responses (SignupRequest, SigninRequest, UserResponse, ProfileResponse) in backend/app/models/schemas.py
- [ ] T010 Create Alembic migration for users, sessions, user_profiles tables with indexes
- [ ] T011 Run Alembic migration to create auth tables
- [ ] T012 [P] Create JWT utility functions: create_access_token, decode_token, verify_password, hash_password in backend/app/utils/auth_utils.py
- [ ] T013 [P] Create auth middleware for JWT token validation (extract token from header, validate, attach user to request) in backend/app/middleware/auth_middleware.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Signup with Background Collection (Priority: P1) 🎯 MVP

**Goal**: Users can create accounts with email/password and provide background information (software/hardware experience, learning goals)

**Independent Test**: POST /api/auth/signup with {"email": "test@example.com", "password": "password123", "name": "Test User", "software_experience": "intermediate", "hardware_experience": "hobbyist", "learning_goals": "Build robots"} returns 201 with user data and session token

### Implementation for User Story 1

- [ ] T014 [P] [US1] Implement user_service: create_user function (hash password, insert user, insert profile) in backend/app/services/user_service.py
- [ ] T015 [P] [US1] Implement user_service: get_user_by_email function in backend/app/services/user_service.py
- [ ] T016 [US1] Implement auth_service: signup function (validate email uniqueness, create user + profile, create session, generate JWT) in backend/app/services/auth_service.py
- [ ] T017 [US1] Create POST /api/auth/signup endpoint with request validation (email format, password length >= 8) in backend/app/api/auth_routes.py
- [ ] T018 [US1] Implement signup flow: validate request → create user + profile → create session → return JWT token
- [ ] T019 [US1] Add error handling: duplicate email (409 Conflict), validation errors (400 Bad Request)
- [ ] T020 [US1] Add password complexity validation: min 8 chars, at least one letter, one number
- [ ] T021 [P] [US1] Create SignupForm.tsx with email, password, name, software_experience dropdown, hardware_experience dropdown, learning_goals textarea in book-source/src/components/Auth/SignupForm.tsx
- [ ] T022 [P] [US1] Create TypeScript interfaces: User, UserProfile, AuthResponse in book-source/src/components/Auth/types.ts
- [ ] T023 [US1] Implement signup API client: POST /api/auth/signup, store JWT in localStorage in book-source/src/components/Auth/SignupForm.tsx
- [ ] T024 [US1] Add form validation: email format, password strength indicator, required fields
- [ ] T025 [US1] Add success/error handling: redirect to home on success, display errors inline

**Checkpoint**: At this point, users should be able to sign up and receive a JWT token (verify in Postgres: check users and user_profiles tables)

---

## Phase 4: User Story 2 - User Signin (Priority: P2) 🎯 MVP

**Goal**: Users can log in with their email and password

**Independent Test**: POST /api/auth/signin with {"email": "test@example.com", "password": "password123"} returns 200 with user data and session token

### Implementation for User Story 2

- [ ] T026 [US2] Implement auth_service: signin function (verify email exists, verify password, create session, generate JWT) in backend/app/services/auth_service.py
- [ ] T027 [US2] Create POST /api/auth/signin endpoint with email and password validation in backend/app/api/auth_routes.py
- [ ] T028 [US2] Implement signin flow: validate credentials → create session → return JWT token
- [ ] T029 [US2] Add error handling: invalid credentials (401 Unauthorized), account not found (404 Not Found)
- [ ] T030 [US2] Implement rate limiting for signin endpoint: max 5 attempts per email per 15 minutes
- [ ] T031 [P] [US2] Create SigninForm.tsx with email and password inputs in book-source/src/components/Auth/SigninForm.tsx
- [ ] T032 [US2] Implement signin API client: POST /api/auth/signin, store JWT in localStorage
- [ ] T033 [US2] Add form validation and error handling: display "Invalid credentials" on 401
- [ ] T034 [US2] Redirect to home page on successful signin

**Checkpoint**: At this point, users should be able to sign in (test with existing user credentials)

---

## Phase 5: User Story 3 - Auth State Management & Protected Routes (Priority: P3) 🎯 MVP

**Goal**: Frontend tracks authentication state and protects routes requiring authentication

**Independent Test**: User signs in → AuthButton shows "Profile" dropdown → User navigates to protected route → Route accessible; User not signed in → Protected route redirects to signin

### Implementation for User Story 3

- [ ] T035 [P] [US3] Create React Context for auth state (user, isAuthenticated, token) in book-source/src/components/Auth/AuthContext.tsx
- [ ] T036 [P] [US3] Implement JWT decoding to extract user info from token in AuthContext
- [ ] T037 [US3] Implement auth state initialization: check localStorage for JWT on app load, validate expiration
- [ ] T038 [US3] Create logout function: clear localStorage, clear auth state
- [ ] T039 [P] [US3] Create AuthButton.tsx: "Login" button when not authenticated, "Profile" dropdown when authenticated in book-source/src/components/Auth/AuthButton.tsx
- [ ] T040 [US3] Integrate AuthButton into Docusaurus navbar (add to theme or custom header component)
- [ ] T041 [P] [US3] Create ProtectedRoute.tsx component: check auth state, redirect to /signin if not authenticated in book-source/src/components/Auth/ProtectedRoute.tsx
- [ ] T042 [US3] Add "Sign Out" option to Profile dropdown that calls logout function

**Checkpoint**: At this point, auth state should be managed (test: sign in → see profile dropdown, sign out → see login button)

---

## Phase 6: User Story 4 - User Profile Management (Priority: P4)

**Goal**: Authenticated users can view and update their profile (background information, learning goals)

**Independent Test**: GET /api/auth/me with valid JWT returns user profile; PUT /api/auth/profile with updated data returns success

### Implementation for User Story 4

- [ ] T043 [US4] Create GET /api/auth/me endpoint (protected): extract user from JWT, return user + profile data in backend/app/api/auth_routes.py
- [ ] T044 [US4] Implement profile_service: get_profile_by_user_id function in backend/app/services/profile_service.py
- [ ] T045 [US4] Implement profile_service: update_profile function in backend/app/services/profile_service.py
- [ ] T046 [US4] Create PUT /api/auth/profile endpoint (protected): validate fields, update profile in backend/app/api/auth_routes.py
- [ ] T047 [US4] Add validation: software_experience enum (beginner/intermediate/advanced), hardware_experience enum (none/hobbyist/professional)
- [ ] T048 [P] [US4] Create ProfileForm.tsx with editable fields (name, software_experience, hardware_experience, learning_goals) in book-source/src/components/Auth/ProfileForm.tsx
- [ ] T049 [US4] Implement profile API client: GET /api/auth/me to load current profile, PUT /api/auth/profile to save changes
- [ ] T050 [US4] Add form validation and success/error handling
- [ ] T051 [US4] Create profile page route /profile using ProtectedRoute wrapper

**Checkpoint**: At this point, users should be able to view and edit their profiles

---

## Phase 7: User Story 5 - Session Management (Priority: P5)

**Goal**: Manage user sessions (logout, session expiration handling)

**Independent Test**: POST /api/auth/signout invalidates session; Expired JWT is rejected on protected endpoints

### Implementation for User Story 5

- [ ] T052 [US5] Create POST /api/auth/signout endpoint (protected): delete session from database in backend/app/api/auth_routes.py
- [ ] T053 [US5] Implement auth_service: signout function (delete session by token) in backend/app/services/auth_service.py
- [ ] T054 [US5] Implement JWT expiration check in auth middleware: return 401 if token expired
- [ ] T055 [US5] Add session validation: check if session exists in database (optional: can skip for stateless JWT)
- [ ] T056 [US5] Implement frontend signout: call POST /api/auth/signout, clear localStorage, clear auth state
- [ ] T057 [US5] Add automatic redirect to /signin on 401 responses (token expired or invalid)
- [ ] T058 [US5] Implement session expiration warning: show modal 5 minutes before expiration, offer to extend session

**Checkpoint**: At this point, session management should be complete (test: sign out → JWT cleared, expired token → 401)

---

## Phase 8: User Story 6 - Protected API Endpoints Integration (Priority: P6)

**Goal**: Integrate auth with existing features (chat history, personalization) - make certain endpoints require authentication

**Independent Test**: Unauthenticated request to /api/chat/history → 401; Authenticated request with valid JWT → 200 with data

### Implementation for User Story 6

- [ ] T059 [P] [US6] Add auth dependency to /api/chat/history endpoint (require JWT token) in backend/app/api/chat_routes.py
- [ ] T060 [P] [US6] Add auth dependency to /api/chat/clear endpoint (require JWT token)
- [ ] T061 [P] [US6] Update chat_sessions table: add user_id column (nullable for backward compatibility) with FK to users table
- [ ] T062 [US6] Update chat message saving: store user_id if authenticated in backend/app/services/db_service.py
- [ ] T063 [US6] Update chat history retrieval: filter by user_id if authenticated
- [ ] T064 [US6] Add migration to add user_id column to chat_sessions table
- [ ] T065 [US6] Frontend: Add JWT token to API requests (Authorization: Bearer <token> header) in book-source/src/components/ChatBot/ChatBot.tsx
- [ ] T066 [US6] Frontend: Handle 401 responses → prompt user to sign in

**Checkpoint**: At this point, chat history should be linked to user accounts (authenticated users only see their own chat history)

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T067 [P] Add comprehensive docstrings (Google style) to all auth services
- [ ] T068 [P] Add OpenAPI documentation tags to all auth endpoints in backend/app/api/auth_routes.py
- [ ] T069 [P] Implement request logging for auth operations (signup, signin, signout) in backend/app/utils/logger.py
- [ ] T070 [P] Add HTTPS enforcement (redirect HTTP to HTTPS in production)
- [ ] T071 [P] Implement CSRF protection for auth endpoints
- [ ] T072 [P] Add email verification flow (optional): send verification email on signup, verify token endpoint
- [ ] T073 [P] Add password reset flow (optional): send reset email, verify reset token, update password
- [ ] T074 [P] Frontend: Add "Remember Me" checkbox (extend JWT expiration to 30 days)
- [ ] T075 [P] Frontend: Add password visibility toggle (eye icon) in signin/signup forms
- [ ] T076 [P] Frontend: Add loading states for auth operations (disable buttons while API calls in flight)
- [ ] T077 [P] Frontend: Add mobile responsive styles for auth forms
- [ ] T078 [P] Create comprehensive README.md with auth setup instructions in backend/README.md
- [ ] T079 [P] Add environment variables documentation for JWT_SECRET in backend/.env.example
- [ ] T080 Run full integration test: signup → signin → access protected route → update profile → signout

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - **US1 (Signup)**: Can start after Foundational
  - **US2 (Signin)**: Depends on US1 (requires user table and auth service)
  - **US3 (Auth State)**: Depends on US2 (requires signin flow)
  - **US4 (Profile)**: Depends on US3 (requires auth state management)
  - **US5 (Session Mgmt)**: Depends on US2 (extends signin/signout)
  - **US6 (Protected APIs)**: Depends on US3 (requires auth middleware)
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (Signup)**: Can start after Foundational - No dependencies on other stories
- **US2 (Signin)**: Depends on US1 completion (requires user creation logic)
- **US3 (Auth State)**: Depends on US2 completion (requires signin flow for testing)
- **US4 (Profile)**: Depends on US3 completion (requires protected routes)
- **US5 (Session Mgmt)**: Can start after US2 - Independent of US3/US4
- **US6 (Protected APIs)**: Depends on US3 completion (requires auth middleware)

### Within Each User Story

- **US1**: User models → auth service (signup) → API endpoint → frontend signup form
- **US2**: Auth service (signin) → API endpoint → frontend signin form
- **US3**: Auth context → AuthButton → ProtectedRoute
- **US4**: Profile service → API endpoints (me, update) → ProfileForm
- **US5**: Signout service → API endpoint → frontend signout + expiration handling
- **US6**: Database migration → protected endpoint decorators → frontend JWT headers

### Parallel Opportunities

- **Phase 1**: T003 (env config), T004 (JWT config), T005 (frontend packages) can run in parallel
- **Phase 2**: T009 (schemas), T012 (JWT utils), T013 (middleware) can run in parallel after T006-T008 complete
- **US1**: T014-T015 (user service functions), T021-T022 (frontend components) can run in parallel
- **US2**: T031 (SigninForm) can run in parallel with T026-T028 (backend signin logic)
- **US3**: T035-T036 (AuthContext), T039 (AuthButton), T041 (ProtectedRoute) can run in parallel
- **US4**: T048 (ProfileForm) can run in parallel with T043-T046 (backend profile logic)
- **US6**: T059-T060 (protect chat endpoints) can run in parallel
- **Phase 9**: Most polish tasks can run in parallel (T067-T079)

---

## Parallel Example: User Story 1 (Signup)

```bash
# Launch parallel tasks for backend and frontend:
Task: "Implement user_service: create_user in backend/app/services/user_service.py"
Task: "Implement user_service: get_user_by_email in backend/app/services/user_service.py"
Task: "Create SignupForm.tsx in book-source/src/components/Auth/SignupForm.tsx"
Task: "Create TypeScript interfaces in book-source/src/components/Auth/types.ts"

# After both complete, connect them:
Task: "Create POST /api/auth/signup endpoint"
Task: "Implement signup API client in SignupForm"
```

---

## Implementation Strategy

### MVP First (US1 + US2 + US3 Only)

**Goal**: Get basic authentication working (signup, signin, auth state)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 (Signup) → Test user creation
4. Complete Phase 4: US2 (Signin) → Test login flow
5. Complete Phase 5: US3 (Auth State) → Test protected routes
6. **STOP and VALIDATE**: Test end-to-end auth flow
7. Deploy/demo if ready

**MVP Scope**: US1 (signup) + US2 (signin) + US3 (auth state) = **Basic authentication** ✅

### Full Feature Set

After MVP validation:

8. Complete Phase 6: US4 (Profile Management)
9. Complete Phase 7: US5 (Session Management)
10. Complete Phase 8: US6 (Protected API Integration)
11. Complete Phase 9: Polish & Cross-Cutting Concerns
12. Final integration testing with RAG chatbot

### Incremental Delivery Milestones

- **Milestone 1**: Setup + Foundational → Infrastructure ready
- **Milestone 2**: + US1 → Users can sign up (test with API)
- **Milestone 3**: + US2 → Users can sign in (test with UI)
- **Milestone 4**: + US3 → **MVP: Auth state managed** 🎯
- **Milestone 5**: + US4 → Profile management
- **Milestone 6**: + US5 → Session management
- **Milestone 7**: + US6 → Chat history linked to users
- **Milestone 8**: + Polish → **Production-ready**

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Focus on MVP first (US1 + US2 + US3) before adding advanced features
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Better Auth React SDK is optional - we're using JWT validation in FastAPI for simplicity

---

## Total Task Count

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 8 tasks
- **Phase 3 (US1 - Signup)**: 12 tasks
- **Phase 4 (US2 - Signin)**: 9 tasks
- **Phase 5 (US3 - Auth State)**: 8 tasks
- **Phase 6 (US4 - Profile)**: 9 tasks
- **Phase 7 (US5 - Session)**: 7 tasks
- **Phase 8 (US6 - Protected APIs)**: 8 tasks
- **Phase 9 (Polish)**: 14 tasks

**Total**: 80 tasks

**MVP Tasks** (US1 + US2 + US3): 5 (setup) + 8 (foundational) + 12 (US1) + 9 (US2) + 8 (US3) = **42 tasks**

**Parallel Opportunities**: 30+ tasks can run in parallel at various stages
