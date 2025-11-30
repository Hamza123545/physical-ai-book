---
id: 0001
title: Better Auth Integration Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: better-auth
branch: main
user: system
command: /sp.plan
labels: ["better-auth", "authentication", "user-profiles", "fastapi", "jwt", "phase-2", "bonus-50-points"]
links:
  spec: null
  ticket: null
  adr: specs/better-auth/plan.md#architectural-decisions
  pr: null
files:
  - specs/better-auth/plan.md
tests:
  - None (planning phase)
---

## Prompt

Create implementation plan for Better Auth integration (Bonus - 50 points).

**Context**: Physical AI textbook backend (FastAPI, Better Auth, Neon Postgres)

**Requirements**:
- Signup/Signin using Better Auth
- At signup: Ask user background questions (software/hardware experience)
- Store user profiles in Postgres
- Protected API endpoints

**Constitution**: Follow .specify/memory/constitution.md (Backend Development Principles)

**Output**: Detailed plan with phases (Auth Setup, User Profile, API Protection, Frontend Integration). Include dependencies, estimated time.

## Response snapshot

Created comprehensive implementation plan (`specs/better-auth/plan.md`) for Better Auth integration following constitution-mandated backend development workflow.

**Key Outputs**:

1. **Technical Context** - Defined stack (Python 3.11+, FastAPI, Better Auth, PyJWT, Neon Postgres, React/TypeScript)
2. **Constitution Check** - All gates PASS (Backend Development Principles, Security Requirements)
3. **Project Structure** - Defined backend/app/{models,api,services,middleware} and book-source/src/components/Auth/
4. **Phase 0: Research** - 7 research tasks (Better Auth integration patterns, JWT validation, user profile schema, session management, protected endpoints, frontend auth state, background questions UX)
5. **Phase 1: Design** - Data models (users, sessions, accounts, user_profiles tables), API contracts (6 auth endpoints), quickstart guide
6. **Architectural Decisions** - 5 ADRs documented:
   - Better Auth with FastAPI JWT validation (vs custom auth, vs Auth0/Clerk)
   - FastAPI JWT validation (vs standalone Better Auth server)
   - Collect background during signup (vs progressive profiling)
   - 7-day session with 24-hour refresh
   - Better Auth's built-in bcrypt hashing

7. **Database Schema**:
   - **users**: id, email, email_verified, name, image, created_at, updated_at
   - **sessions**: id, user_id, expires_at, session_token, created_at
   - **accounts**: id, user_id, type, provider, tokens (OAuth - future)
   - **user_profiles**: id, user_id, software_experience, hardware_experience, learning_goals, preferred_topics, current_chapter, completed_lessons

8. **API Endpoints Designed**:
   - POST /api/auth/signup (with background questions)
   - POST /api/auth/signin
   - POST /api/auth/signout
   - GET /api/auth/me (protected)
   - PUT /api/auth/profile (protected)
   - GET /api/auth/session

9. **Frontend Components**:
   - SignupForm.tsx (with background questions)
   - SigninForm.tsx
   - AuthButton.tsx (header login/profile dropdown)
   - ProfileForm.tsx (profile editor)
   - ProtectedRoute.tsx (route guard)
   - UserProfileModal.tsx (background questions modal)

10. **Integration Points**:
    - RAG Chatbot: Link chat sessions to user_id, personalized responses
    - Content Personalization: Profile-based recommendations, adaptive difficulty
    - Urdu Translation: User language preference, per-user cache

11. **Success Criteria**:
    - Functional: Signup/signin work, protected endpoints require JWT, profiles stored
    - Performance: JWT validation < 100ms, signup < 2s, signin < 1s
    - Quality: 70% test coverage, password/email validation
    - Security: Bcrypt hashing, JWT secret in env, HTTPS, token invalidation

12. **Timeline**: 12-16 hours (1.5-2 days)

**Next Steps**:
1. Run `/sp.tasks` to generate tasks.md
2. Execute Phase 0 research
3. Execute Phase 1 design (data-model.md, contracts/auth-api-spec.yaml, quickstart.md)
4. Begin implementation via `/sp.implement`

## Outcome

- ✅ Impact: Comprehensive plan created for 50-point bonus hackathon requirement (Better Auth integration)
- 🧪 Tests: None (planning phase - tests defined in plan)
- 📁 Files: specs/better-auth/plan.md (650+ lines)
- 🔁 Next prompts: `/sp.tasks` → Phase 0 research → Phase 1 design → `/sp.implement`
- 🧠 Reflection: Plan follows constitution Backend Development Principles. Hybrid approach (Better Auth React SDK + FastAPI JWT validation) simplifies deployment. Background questions collected at signup enable personalization features. Clear integration path with RAG chatbot (user_id linking). Realistic timeline (12-16 hours).

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Execute `/sp.tasks` to validate plan completeness by generating actionable task breakdown
