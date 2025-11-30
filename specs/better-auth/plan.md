# Implementation Plan: Better Auth Integration

**Branch**: `main` | **Date**: 2025-11-29 | **Feature**: Better Auth (Bonus - 50 points)
**Input**: Hackathon requirements for user authentication with background collection

## Summary

Integrate Better Auth into the Physical AI textbook backend to enable user authentication with profile collection. The system will:
- Provide signup/signin functionality using Better Auth
- Collect user background information during signup (software/hardware experience)
- Store user profiles in Neon Postgres
- Protect API endpoints requiring authentication (personalization, chat history)
- Enable future features (content personalization, progress tracking)

**Technical Approach**: Better Auth (TypeScript auth library) with FastAPI backend integration, Neon Postgres for user storage, React components in Docusaurus for auth UI.

## Technical Context

**Language/Version**: Python 3.11+ (FastAPI backend), TypeScript/Node.js (Better Auth server), React 18 (Docusaurus frontend)
**Primary Dependencies**: Better Auth, FastAPI, SQLAlchemy, Pydantic, React, better-auth React SDK
**Storage**: Neon Postgres (users, sessions, user_profiles tables)
**Testing**: pytest (backend), Jest (frontend auth components)
**Target Platform**: Cloud-deployed API (Vercel/Railway/Render) + Static Docusaurus site (GitHub Pages/Netlify)
**Project Type**: Web (backend API + auth middleware + frontend components)
**Performance Goals**:
- Auth token validation: < 100ms
- User profile retrieval: < 200ms
- Signup flow: < 2 seconds end-to-end
**Constraints**:
- Must integrate with existing FastAPI backend
- Must work with existing Docusaurus frontend
- No breaking changes to existing unauthenticated features
- Session management compatible with stateless API
**Scale/Scope**:
- 100s of users (hackathon scale)
- User profiles: ~10-15 fields (name, email, software_experience, hardware_experience, learning_goals, etc.)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Phase-Driven Architecture**: This is Phase 2 (RAG + Features) - Better Auth is bonus feature
✅ **Spec-Driven Development**: Following `/sp.plan` → `/sp.implement` workflow (backend development path)
✅ **Backend Development Principles**: Using FastAPI, Neon Postgres, proper error handling, security standards
✅ **Code Quality Standards**: Python 3.11+ with type hints, comprehensive docstrings, test coverage > 70%
✅ **API Design Standards**: RESTful conventions, consistent response formats, proper HTTP status codes
✅ **Security Requirements**: Environment variables for secrets, JWT token validation, session security, HTTPS enforcement
✅ **File Organization**: Backend follows `backend/app/{main,config,models,api,services,utils}` structure

**No violations detected. All gates PASS.**

## Project Structure

### Documentation (this feature)

```text
specs/better-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0: Better Auth integration patterns, JWT validation
├── data-model.md        # Phase 1: Database schemas (users, sessions, profiles)
├── quickstart.md        # Phase 1: Setup instructions for Better Auth
├── contracts/           # Phase 1: API contracts (auth endpoints)
│   └── auth-api-spec.yaml
└── tasks.md             # Phase 2: Created by /sp.tasks command
```

### Source Code (repository root)

```text
backend/                          # FastAPI backend
├── app/
│   ├── main.py                  # FastAPI app (add auth middleware)
│   ├── config.py                # Add Better Auth config (JWT secret, etc.)
│   ├── models/
│   │   ├── user.py              # SQLAlchemy User model
│   │   ├── user_profile.py      # SQLAlchemy UserProfile model
│   │   └── session.py           # SQLAlchemy Session model (Better Auth)
│   ├── api/
│   │   ├── auth_routes.py       # /api/auth endpoints (signup, signin, signout, profile)
│   │   └── protected_routes.py  # Example protected endpoints
│   ├── services/
│   │   ├── auth_service.py      # Better Auth integration, JWT validation
│   │   ├── user_service.py      # User CRUD operations
│   │   └── profile_service.py   # User profile operations
│   ├── middleware/
│   │   └── auth_middleware.py   # JWT token validation middleware
│   └── utils/
│       └── auth_utils.py        # JWT decode, token validation helpers
├── tests/
│   ├── unit/
│   │   ├── test_auth_service.py
│   │   └── test_user_service.py
│   └── integration/
│       └── test_auth_api.py
└── alembic/
    └── versions/
        └── 0002_create_auth_tables.py  # Migration for users, sessions, profiles

auth-server/                      # Better Auth TypeScript server (optional)
├── src/
│   ├── index.ts                 # Better Auth server setup
│   └── config.ts                # Better Auth configuration
├── package.json
└── tsconfig.json

book-source/                      # Existing Docusaurus site
└── src/
    └── components/
        ├── Auth/
        │   ├── SignupForm.tsx   # Signup form with background questions
        │   ├── SigninForm.tsx   # Signin form
        │   ├── AuthButton.tsx   # Header auth button (Login/Profile dropdown)
        │   ├── ProfileForm.tsx  # User profile editor
        │   ├── ProtectedRoute.tsx # Route guard component
        │   └── types.ts         # TypeScript interfaces
        └── UserProfileModal.tsx # Modal for background questions during signup
```

**Structure Decision**: Hybrid approach - Better Auth can run as standalone TypeScript server OR integrated directly into FastAPI via JWT validation. For hackathon simplicity, we'll use **JWT validation in FastAPI** (no separate auth server), with Better Auth React SDK for frontend.

## Complexity Tracking

> No violations detected - this section is empty.

---

## Phase 0: Research & Technology Decisions

**Prerequisites**: Constitution approved
**Output**: `research.md` with all technology choices documented

### Research Tasks

1. **Better Auth Integration Patterns**
   - Research: Better Auth with FastAPI (JWT validation approach vs full server integration)
   - Decision needed: Standalone Better Auth server vs FastAPI JWT validation
   - Reference: Better Auth documentation, FastAPI auth patterns

2. **JWT Token Validation in FastAPI**
   - Research: Libraries for JWT validation (PyJWT, python-jose)
   - Decision needed: Token validation strategy, middleware vs dependency injection
   - Reference: FastAPI security documentation, JWT best practices

3. **User Profile Schema Design**
   - Research: Better Auth default user schema, custom fields extension
   - Decision needed: Profile fields (software_experience, hardware_experience, learning_goals, preferred_topics)
   - Reference: User onboarding best practices, progressive profiling

4. **Session Management Strategy**
   - Research: Better Auth session handling, JWT refresh tokens
   - Decision needed: Session expiration (24 hours? 7 days?), refresh token strategy
   - Reference: OAuth 2.0 best practices, session security

5. **Protected Endpoint Patterns**
   - Research: FastAPI dependency injection for auth, route decorators
   - Decision needed: How to mark endpoints as protected, optional vs required auth
   - Reference: FastAPI dependencies, auth middleware patterns

6. **Frontend Auth State Management**
   - Research: Better Auth React SDK, React Context for auth state
   - Decision needed: Where to store auth state (localStorage, React Context, both)
   - Reference: React auth patterns, security best practices for SPAs

7. **Background Questions UI/UX**
   - Research: Multi-step signup flows, progressive profiling
   - Decision needed: Ask during signup vs after first login, required vs optional fields
   - Reference: UX best practices for user onboarding

### Research Outputs

All findings documented in `specs/better-auth/research.md` with format:
```markdown
## Decision: [Technology/Pattern Choice]
**Rationale**: [Why chosen]
**Alternatives Considered**: [What else evaluated]
**Trade-offs**: [Pros and cons]
**References**: [Documentation links]
```

---

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete
**Output**: `data-model.md`, `contracts/auth-api-spec.yaml`, `quickstart.md`

### 1.1 Data Model Design (`data-model.md`)

**Postgres Schema (Neon)**:

```sql
-- Users table (Better Auth compatible schema)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    name VARCHAR(255),
    image TEXT,  -- Profile picture URL (optional)
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Sessions table (Better Auth session management)
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    expires_at TIMESTAMP NOT NULL,
    session_token TEXT UNIQUE NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Accounts table (Better Auth OAuth providers - optional for future)
CREATE TABLE accounts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    type VARCHAR(50) NOT NULL,  -- 'email', 'oauth'
    provider VARCHAR(50) NOT NULL,  -- 'email', 'google', 'github'
    provider_account_id VARCHAR(255),
    refresh_token TEXT,
    access_token TEXT,
    expires_at TIMESTAMP,
    token_type VARCHAR(50),
    scope TEXT,
    id_token TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);

-- User profiles (custom fields for Physical AI textbook)
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    software_experience VARCHAR(50),  -- 'beginner', 'intermediate', 'advanced'
    hardware_experience VARCHAR(50),  -- 'none', 'hobbyist', 'professional'
    learning_goals TEXT,  -- Free text: "I want to build humanoid robots"
    preferred_topics JSONB,  -- ["ros2", "gazebo", "isaac", "vla"]
    current_chapter INTEGER,  -- Progress tracking
    completed_lessons JSONB,  -- ["chapter-01-lesson-01", "chapter-01-lesson-02"]
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(user_id)
);

-- Indexes
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_sessions_user ON sessions(user_id);
CREATE INDEX idx_sessions_token ON sessions(session_token);
CREATE INDEX idx_user_profiles_user ON user_profiles(user_id);
```

**Better Auth Configuration Schema**:

```typescript
// Better Auth config (if using standalone server)
export const authConfig = {
  database: {
    provider: "postgresql",
    url: process.env.DATABASE_URL,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,  // Optional: enable later
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7,  // 7 days
    updateAge: 60 * 60 * 24,  // Update session every 24 hours
  },
  // Custom fields for user profiles
  user: {
    additionalFields: {
      software_experience: {
        type: "string",
        required: false,
      },
      hardware_experience: {
        type: "string",
        required: false,
      },
    },
  },
};
```

### 1.2 API Contracts (`contracts/auth-api-spec.yaml`)

**Endpoints**:

1. **POST /api/auth/signup** - User registration with background questions
   ```yaml
   /api/auth/signup:
     post:
       summary: Register new user with background profile
       requestBody:
         content:
           application/json:
             schema:
               type: object
               required: [email, password, name]
               properties:
                 email:
                   type: string
                   format: email
                 password:
                   type: string
                   minLength: 8
                 name:
                   type: string
                 software_experience:
                   type: string
                   enum: [beginner, intermediate, advanced]
                 hardware_experience:
                   type: string
                   enum: [none, hobbyist, professional]
                 learning_goals:
                   type: string
                   maxLength: 500
       responses:
         201:
           content:
             application/json:
               schema:
                 type: object
                 properties:
                   success:
                     type: boolean
                   data:
                     type: object
                     properties:
                       user:
                         $ref: '#/components/schemas/User'
                       session_token:
                         type: string
   ```

2. **POST /api/auth/signin** - User login
   ```yaml
   /api/auth/signin:
     post:
       summary: Authenticate user and create session
       requestBody:
         content:
           application/json:
             schema:
               type: object
               required: [email, password]
               properties:
                 email:
                   type: string
                   format: email
                 password:
                   type: string
       responses:
         200:
           content:
             application/json:
               schema:
                 type: object
                 properties:
                   success: boolean
                   data:
                     type: object
                     properties:
                       user:
                         $ref: '#/components/schemas/User'
                       session_token:
                         type: string
   ```

3. **POST /api/auth/signout** - Logout (invalidate session)
4. **GET /api/auth/me** - Get current user profile (protected)
5. **PUT /api/auth/profile** - Update user profile (protected)
6. **GET /api/auth/session** - Validate session token

Full OpenAPI spec generated in `contracts/auth-api-spec.yaml`.

### 1.3 Quickstart Guide (`quickstart.md`)

Step-by-step developer setup:
1. Install Better Auth dependencies
2. Configure Better Auth with Neon Postgres
3. Run database migrations (users, sessions, accounts, user_profiles)
4. Setup JWT secret in environment variables
5. Implement auth middleware in FastAPI
6. Create signup/signin API endpoints
7. Integrate Better Auth React SDK in Docusaurus
8. Test auth flow (signup → signin → protected endpoint → signout)

### 1.4 Agent Context Update

Run constitution-mandated agent context update:
```powershell
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

Adds Better Auth technologies to agent context:
- Better Auth
- PyJWT / python-jose
- FastAPI security utilities
- Better Auth React SDK

---

## Phase 2: Task Breakdown

**Note**: This phase is executed by `/sp.tasks` command, NOT by `/sp.plan`.

The `/sp.tasks` command will generate `specs/better-auth/tasks.md` with detailed implementation tasks based on this plan.

**Expected Task Structure**:
1. Database Schema & Migrations (users, sessions, accounts, user_profiles tables)
2. Better Auth Configuration (JWT secret, session expiration, email/password provider)
3. User Service (CRUD operations for users and profiles)
4. Auth Middleware (JWT token validation, protected route decorator)
5. Auth API Endpoints (signup, signin, signout, me, profile update)
6. Frontend Signup Form (with background questions)
7. Frontend Signin Form
8. Auth State Management (React Context, localStorage)
9. Protected Route Component
10. Testing (Unit tests for auth service, integration tests for auth endpoints)
11. Documentation (API docs, user guide)

---

## Phase 3-7: Implementation Phases

**Note**: These phases are executed by `/sp.implement` command, NOT by `/sp.plan`.

The `/sp.implement` command will execute tasks from `tasks.md` in dependency order, implementing:
- Phase 3: Database & Better Auth Setup
- Phase 4: Backend Auth Services
- Phase 5: Auth Middleware & Protected Endpoints
- Phase 6: Frontend Auth Components
- Phase 7: Testing & Documentation

---

## Architectural Decisions

### ADR-001: Better Auth vs Custom Auth Implementation
**Decision**: Use Better Auth (TypeScript library) with FastAPI JWT validation
**Rationale**: Better Auth provides production-ready auth with minimal setup. We'll use JWT validation in FastAPI rather than running a separate auth server to reduce infrastructure complexity.
**Alternatives**:
- Custom auth (rejected: requires implementing password hashing, session management, security best practices from scratch)
- Auth0/Clerk (rejected: paid services, overkill for hackathon)

### ADR-002: Standalone Auth Server vs FastAPI JWT Validation
**Decision**: FastAPI JWT validation (no separate Better Auth server)
**Rationale**: Simpler deployment, fewer moving parts. Better Auth React SDK handles frontend auth flow, FastAPI validates JWT tokens.
**Alternatives**:
- Standalone Better Auth server (rejected: adds infrastructure complexity, separate deployment)

### ADR-003: User Profile Collection Timing
**Decision**: Collect background questions during signup (required fields)
**Rationale**: Ensures complete profiles for personalization features. Users provide context upfront, enabling better recommendations.
**Alternatives**:
- Progressive profiling (ask after first login) - rejected: adds friction, delays personalization
- Optional fields - rejected: incomplete data reduces personalization quality

### ADR-004: Session Expiration Strategy
**Decision**: 7-day session with 24-hour refresh
**Rationale**: Balances security (not indefinite) with UX (users don't need to login daily). Session updates every 24 hours if user is active.
**Alternatives**:
- 24-hour expiration (rejected: too frequent, poor UX)
- 30-day expiration (rejected: security risk for hackathon demo)

### ADR-005: Password Storage
**Decision**: Use Better Auth's built-in bcrypt hashing
**Rationale**: Industry-standard password hashing, automatically salted, properly implemented.
**Alternatives**: Custom hashing (rejected: security risk, reinventing the wheel)

---

## Dependencies

### External Services
- **Neon Postgres**: Free tier (3GB storage, shared compute) - already provisioned for RAG chatbot
- **Better Auth**: Open-source auth library (npm package)

### Python Packages (backend/requirements.txt additions)
```
python-jose[cryptography]==3.3.0  # JWT validation
passlib[bcrypt]==1.7.4  # Password hashing (if needed)
python-multipart==0.0.6  # Form data parsing
```

### TypeScript/Node Packages (if using standalone auth server)
```json
{
  "dependencies": {
    "better-auth": "^0.x.x",
    "pg": "^8.11.3",
    "@types/node": "^20.x.x",
    "typescript": "^5.x.x"
  }
}
```

### Frontend Packages (book-source/package.json additions)
```json
{
  "dependencies": {
    "better-auth": "^0.x.x",  // Better Auth React SDK
    "jwt-decode": "^4.0.0"  // Decode JWT tokens client-side
  }
}
```

---

## Success Criteria

✅ **Functional**:
- [ ] Users can signup with email/password + background questions
- [ ] Users can signin and receive session token (JWT)
- [ ] Protected endpoints require valid JWT token (401 if missing/invalid)
- [ ] User profiles stored in Postgres with background info
- [ ] Frontend displays auth state (logged in vs logged out)
- [ ] Users can update their profile

✅ **Performance**:
- [ ] JWT token validation < 100ms
- [ ] Signup flow < 2 seconds
- [ ] Signin flow < 1 second

✅ **Quality**:
- [ ] Test coverage > 70% (auth service, user service)
- [ ] Integration tests for all auth endpoints
- [ ] Password validation (min 8 chars, complexity rules)
- [ ] Email validation

✅ **Security**:
- [ ] Passwords hashed with bcrypt (never stored plain text)
- [ ] JWT secret stored in environment variable
- [ ] HTTPS enforced (in production)
- [ ] Session tokens invalidated on logout
- [ ] Input validation on all auth endpoints

✅ **Documentation**:
- [ ] API documentation for auth endpoints (Swagger UI)
- [ ] User guide for signup/signin flow
- [ ] Environment variables documented in .env.example

---

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Better Auth compatibility issues with FastAPI | Low | Medium | Use JWT validation approach (language-agnostic) |
| Session token leakage | Low | High | HTTPS enforcement, secure cookie flags, token expiration |
| Incomplete user profiles | Medium | Medium | Make background questions required during signup |
| Password reset complexity | Low | Low | Out of scope for hackathon (can add later via Better Auth) |
| Auth state sync issues (frontend/backend) | Medium | Medium | Use Better Auth React SDK, test thoroughly |

---

## Estimated Timeline

- **Phase 0 (Research)**: 1 hour
- **Phase 1 (Design & Contracts)**: 2 hours
- **Phase 2 (Task Breakdown)**: 30 minutes (/sp.tasks)
- **Phase 3-7 (Implementation)**: 8-12 hours (/sp.implement)
  - Database migrations: 1 hour
  - Backend auth services: 3-4 hours
  - Auth middleware: 2 hours
  - Frontend components: 3-4 hours
  - Testing: 2 hours
- **Total**: ~12-16 hours (1.5-2 days of focused work)

---

## Integration with Other Features

### RAG Chatbot Integration
- **Authenticated Chat History**: Link chat sessions to user_id (currently anonymous)
- **Personalized Responses**: Use user profile (software/hardware experience) to tailor explanations
- **API Changes**: Add optional user authentication to `/api/chat` endpoints

### Content Personalization (Future Bonus Feature)
- **Profile-Based Recommendations**: Suggest lessons based on software/hardware experience
- **Adaptive Difficulty**: Adjust CEFR level based on user background
- **Progress Tracking**: Store completed_lessons in user_profiles

### Urdu Translation (Future Bonus Feature)
- **User Language Preference**: Add `preferred_language` to user_profiles
- **Translation Cache per User**: Cache translated content for authenticated users

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown in `specs/better-auth/tasks.md`
2. Execute Phase 0 research tasks to resolve Better Auth integration patterns
3. Execute Phase 1 design tasks to generate data-model.md and auth API contracts
4. Begin implementation via `/sp.implement` command

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` command
**Constitution Compliance**: ✅ PASS
**Output**: `plan.md` (this file), `research.md` (to be generated in Phase 0)
