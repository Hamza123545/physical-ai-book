# Tasks: Content Personalization

**Created Using**: SpecKit Plus + Claude Code  
**Implemented By**: Claude (Anthropic) + SpecKit Plus  
**Completion Date**: 2025-12-01  
**Input**: Design documents from `/specs/content-personalization/`
**Prerequisites**: plan.md (created), spec.md (requirements documented in plan)

**Tests**: Not explicitly requested - focused on implementation and integration

**Organization**: Tasks are grouped by functional capability (user story) to enable independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (FastAPI structure)
- **Frontend**: `book-source/src/components/Personalization/`
- Paths follow web app structure as defined in plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and personalization infrastructure setup

- [x] T001 Create personalization directory structure: api.physical_book/app/models/{personalized_content_cache,user_background}.py
- [x] T002 Add markdown-it-py, mdurl to api.physical_book/requirements.txt (using LiteLLM for Gemini)
- [x] T003 [P] Create Gemini config for personalization (reuse existing LiteLLM client from RAG chatbot)
- [x] T004 [P] Add react-markdown, remark-gfm to book-source/package.json for rendering personalized content

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core personalization infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create PersonalizedContentCache model: personalized_content_cache table with chapter_id, cache_key, original_content, personalized_content, model_used, tokens_used, generation_time_ms in api.physical_book/app/models/personalized_content_cache.py
- [x] T006 Create UserBackground model: user_backgrounds table with user_id, software_experience, hardware_experience, robotics_experience, current_role, programming_languages, learning_goals, industry in api.physical_book/app/models/user_background.py
- [x] T007 [P] Create Pydantic schemas for personalization requests/responses (PersonalizeRequest, PersonalizeResponse) in api.physical_book/app/models/schemas.py
- [x] T008 Create Alembic migration for personalized_content_cache, user_backgrounds tables with indexes
- [x] T009 Run Alembic migration to create personalization tables
- [x] T010 [P] Create profile hashing utility: _generate_cache_key function (MD5 hash) in api.physical_book/app/services/personalization_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Profile-Based Content Personalization (Priority: P1) 🎯 MVP

**Goal**: Personalize lesson content based on user's software/hardware experience using OpenAI GPT-4

**Independent Test**: POST /api/personalize/chapter with {"chapter_id": "chapter-01", "lesson_id": "lesson-01", "user_profile": {"software_experience": "beginner", "hardware_experience": "none"}} returns personalized Markdown content adapted for beginners

### Implementation for User Story 1

- [x] T011 [P] [US1] Create Markdown processor: strip frontmatter and custom components in api.physical_book/app/services/personalization_service.py
- [x] T012 [P] [US1] Implement MDX structure preservation: strip_frontmatter() and strip_custom_components() functions in personalization_service.py
- [x] T013 [P] [US1] Create personalization prompt template for Gemini: adapt language/examples/analogies based on profile in api.physical_book/app/services/personalization_service.py
- [x] T014 [US1] Implement content_adapter: call Gemini via LiteLLM with personalization prompt, validate output in api.physical_book/app/services/personalization_service.py
- [x] T015 [US1] Add output validation: ensure headers preserved, code blocks unchanged, no broken MDX (via cleaning functions)
- [x] T016 [US1] Implement profile-based adaptation logic: map software/hardware experience to adaptation strategy (beginner → simple analogies, advanced → technical depth)
- [x] T017 [US1] Create cache_service: check cache by cache_key, store new personalizations in api.physical_book/app/services/personalization_service.py
- [x] T018 [US1] Implement personalization_service: orchestrate cache check → content adapter → cache store in api.physical_book/app/services/personalization_service.py
- [x] T019 [US1] Create POST /api/personalize endpoint with chapter_id, user_profile validation in api.physical_book/app/api/content_routes.py
- [x] T020 [US1] Implement personalization flow: validate request → generate cache_key → check cache → adapt content (if cold) → return personalized content
- [x] T021 [US1] Add error handling: Gemini API failures (fallback to original content), MDX parsing errors
- [x] T022 [US1] Add token usage tracking and cost monitoring for personalization requests

**Checkpoint**: At this point, personalization should work (test with API: send user profile + lesson → receive adapted content)

---

## Phase 4: User Story 2 - Caching & Performance Optimization (Priority: P2) 🎯 MVP

**Goal**: Cache personalized content to reduce OpenAI costs and improve response times (< 500ms for cached content)

**Independent Test**: First personalization takes ~10 seconds (cold), subsequent requests with same profile_hash return < 500ms (warm)

### Implementation for User Story 2

- [x] T023 [US2] Implement cache lookup: query personalized_content_cache by (chapter_id, cache_key) in api.physical_book/app/services/personalization_service.py
- [x] T024 [US2] Implement cache storage: insert personalized content with 7-day expiration check in _check_cache()
- [x] T025 [US2] Add cache hit/miss tracking: return cache_hit flag in personalize_content() response
- [ ] T026 [US2] Implement cache expiration cleanup: background job to delete expired entries (optional, or rely on DB auto-delete)
- [ ] T027 [US2] Add cache statistics endpoint: GET /api/personalize/stats (cache hit rate, avg generation time, total personalizations)
- [x] T028 [US2] Optimize database queries: proper indexing on (chapter_id, cache_key) via Alembic migration

**Checkpoint**: At this point, caching should work (test: personalize same lesson twice with same profile → second request < 500ms)

---

## Phase 5: User Story 3 - Frontend Personalization UI (Priority: P3) 🎯 MVP

**Goal**: Users see a "Personalize for Me" button at the start of each chapter and can view personalized content

**Independent Test**: Visit a chapter page → see "Personalize for Me" button → click → loading indicator → see personalized content → toggle back to original

### Implementation for User Story 3

- [x] T029 [P] [US3] Create PersonalizeButton.tsx component with chapter_id props in book-source/src/components/PersonalizeButton.tsx
- [x] T030 [P] [US3] Create personalized content rendering in DocItem/Content wrapper using ReactMarkdown in book-source/src/theme/DocItem/Content/index.tsx
- [x] T031 [P] [US3] Create toggle functionality to switch between original/personalized views in DocItem/Content/index.tsx
- [x] T032 [P] [US3] Create loading state with progress message "Personalizing content for you..." in DocItem/Content/index.tsx
- [x] T033 [P] [US3] Create TypeScript interfaces: PersonalizeRequest, PersonalizeResponse in API service files
- [x] T034 [US3] Implement personalization API client: POST /api/personalize with user profile from AuthContext
- [x] T035 [US3] Implement state management: original content, personalized content, showPersonalized flag, isLoading flag
- [x] T036 [US3] Add PersonalizeButton to DocItem/Content wrapper (integrated into Docusaurus theme)
- [x] T037 [US3] Implement personalization flow: click button → fetch profile from auth context → call API → render personalized content
- [x] T038 [US3] Implement toggle functionality: switch between original and personalized views (preserve both in state)
- [x] T039 [US3] Add error handling: display error message if API fails, fallback to original content
- [x] T040 [US3] Add success indicator: show metadata badge (cached/fresh) when showing personalized content

**Checkpoint**: At this point, personalization UI should work (test: click button → see loading → see adapted content → toggle back)

---

## Phase 6: User Story 4 - Advanced Personalization Features (Priority: P4)

**Goal**: Add preview, status check, and cache management endpoints

**Independent Test**: GET /api/personalize/status/{chapter_id}/{lesson_id} returns cache status; POST /api/personalize/preview returns personalized content without caching

### Implementation for User Story 4

- [ ] T041 [P] [US4] Create GET /api/personalize/status/{chapter_id}/{lesson_id} endpoint: check if personalized version exists for given profile in backend/app/api/personalization_routes.py
- [ ] T042 [P] [US4] Create POST /api/personalize/preview endpoint: personalize content without caching (for testing) in backend/app/api/personalization_routes.py
- [ ] T043 [US4] Create DELETE /api/personalize/cache/clear endpoint (admin): clear cache for specific chapter or all chapters
- [ ] T044 [US4] Implement cache invalidation logic: delete by chapter_id or clear all
- [ ] T045 [US4] Add personalization history tracking: store which lessons user has personalized (in user_profiles table or separate table)
- [ ] T046 [US4] Frontend: Add "Preview Personalization" mode (call preview endpoint instead of regular endpoint)
- [ ] T047 [US4] Frontend: Show cache status indicator: "Cached" or "Generating..." before personalization

**Checkpoint**: At this point, advanced features should work (test: check status, preview, clear cache)

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T048 [P] Add comprehensive docstrings (Google style) to all personalization services
- [ ] T049 [P] Add OpenAPI documentation tags to all personalization endpoints
- [ ] T050 [P] Implement request logging for personalization operations (profile_hash, generation time, cache hit/miss)
- [ ] T051 [P] Add input sanitization for user profiles (prevent prompt injection)
- [ ] T052 [P] Optimize OpenAI prompts: reduce token usage while maintaining quality
- [ ] T053 [P] Add A/B testing framework: randomly serve original vs personalized to measure effectiveness (optional)
- [ ] T054 [P] Implement user feedback mechanism: "Was this personalization helpful?" thumbs up/down
- [ ] T055 [P] Frontend: Add animation for personalization transition (fade in/out)
- [ ] T056 [P] Frontend: Add mobile responsive styles for personalization UI
- [ ] T057 [P] Frontend: Add keyboard shortcuts (P to personalize, O for original)
- [ ] T058 [P] Create README.md with personalization setup instructions in backend/README.md
- [ ] T059 [P] Add environment variables documentation for personalization settings
- [ ] T060 Run full integration test: authenticate → personalize lesson → verify content adapted → toggle original → clear cache

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - **US1 (Personalization)**: Can start after Foundational
  - **US2 (Caching)**: Depends on US1 (requires personalization service)
  - **US3 (Frontend UI)**: Depends on US1 (requires API endpoints)
  - **US4 (Advanced Features)**: Depends on US2 (extends caching logic)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (Personalization)**: Can start after Foundational - No dependencies on other stories
- **US2 (Caching)**: Depends on US1 completion (requires content_adapter service)
- **US3 (Frontend UI)**: Depends on US1 completion (requires POST /api/personalize/chapter endpoint)
- **US4 (Advanced Features)**: Depends on US2 completion (extends caching)

### Within Each User Story

- **US1**: Markdown processor → prompt templates → content adapter → cache service → personalization service → API endpoint
- **US2**: Cache lookup → cache storage → cache tracking → cache cleanup → cache stats
- **US3**: React components → API client → state management → integration into chapter pages
- **US4**: Status endpoint → preview endpoint → cache clear endpoint → frontend features

### Parallel Opportunities

- **Phase 1**: T003 (OpenAI config), T004 (frontend packages) can run in parallel
- **Phase 2**: T007 (schemas), T010 (profile hasher) can run in parallel after T005-T006 complete
- **US1**: T011-T012 (Markdown processor), T013 (prompt templates) can run in parallel
- **US2**: T023-T024 (cache operations) can run in parallel
- **US3**: T029-T033 (all React components and types) can run in parallel
- **US4**: T041-T042 (status and preview endpoints) can run in parallel
- **Phase 7**: Most polish tasks can run in parallel (T048-T059)

---

## Parallel Example: User Story 3 (Frontend UI)

```bash
# Launch all frontend components in parallel:
Task: "Create PersonalizeButton.tsx in book-source/src/components/Personalization/PersonalizeButton.tsx"
Task: "Create PersonalizedContent.tsx in book-source/src/components/Personalization/PersonalizedContent.tsx"
Task: "Create PersonalizationToggle.tsx in book-source/src/components/Personalization/PersonalizationToggle.tsx"
Task: "Create LoadingIndicator.tsx in book-source/src/components/Personalization/LoadingIndicator.tsx"
Task: "Create TypeScript interfaces in book-source/src/components/Personalization/types.ts"

# After all complete, integrate them:
Task: "Implement personalization API client and state management"
Task: "Add PersonalizeButton to chapter header component"
```

---

## Implementation Strategy

### MVP First (US1 + US2 + US3 Only)

**Goal**: Get content personalization working with caching and UI

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 (Personalization) → Test with API (Postman/curl)
4. Complete Phase 4: US2 (Caching) → Verify cache hit rate
5. Complete Phase 5: US3 (Frontend UI) → Test in browser
6. **STOP and VALIDATE**: Test end-to-end personalization flow
7. Deploy/demo if ready

**MVP Scope**: US1 (personalize) + US2 (cache) + US3 (UI) = **Working personalization** ✅

### Full Feature Set

After MVP validation:

8. Complete Phase 6: US4 (Advanced Features)
9. Complete Phase 7: Polish & Cross-Cutting Concerns
10. Final integration testing with Better Auth (user profiles)

### Incremental Delivery Milestones

- **Milestone 1**: Setup + Foundational → Infrastructure ready
- **Milestone 2**: + US1 → Personalization works (test with API)
- **Milestone 3**: + US2 → Caching works (80%+ hit rate)
- **Milestone 4**: + US3 → **MVP: Personalization UI functional** 🎯
- **Milestone 5**: + US4 → Advanced features
- **Milestone 6**: + Polish → **Production-ready**

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Focus on MVP first (US1 + US2 + US3) before adding advanced features
- Requires Better Auth integration to retrieve user profiles
- OpenAI costs estimated: ~$40-50 for 39 lessons × 5 profile variations (mitigated by 80%+ cache hit rate)

---

## Total Task Count

- **Phase 1 (Setup)**: 4 tasks
- **Phase 2 (Foundational)**: 6 tasks
- **Phase 3 (US1 - Personalization)**: 12 tasks
- **Phase 4 (US2 - Caching)**: 6 tasks
- **Phase 5 (US3 - Frontend UI)**: 12 tasks
- **Phase 6 (US4 - Advanced Features)**: 7 tasks
- **Phase 7 (Polish)**: 13 tasks

**Total**: 60 tasks

**MVP Tasks** (US1 + US2 + US3): 4 (setup) + 6 (foundational) + 12 (US1) + 6 (US2) + 12 (US3) = **40 tasks**

**Parallel Opportunities**: 25+ tasks can run in parallel at various stages
