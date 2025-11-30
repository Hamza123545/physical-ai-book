# Tasks: Urdu Translation

**Input**: Design documents from `/specs/urdu-translation/`
**Prerequisites**: plan.md (created), spec.md (requirements documented in plan)

**Tests**: Not explicitly requested - focused on implementation and integration

**Organization**: Tasks are grouped by functional capability (user story) to enable independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (FastAPI structure)
- **Frontend**: `book-source/src/components/Translation/`
- Paths follow web app structure as defined in plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and translation infrastructure setup

- [ ] T001 Create translation directory structure: backend/app/models/{translated_content,translation_request}.py
- [ ] T002 Add markdown-it-py, mdurl to backend/requirements.txt (reuse from embeddings if already present)
- [ ] T003 [P] Create OpenAI config for translation (reuse existing OpenAI client from RAG chatbot)
- [ ] T004 [P] Add react-markdown, remark-gfm to book-source/package.json for rendering Urdu content
- [ ] T005 [P] Create technical terms list: comprehensive list of terms to preserve (ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, Python, URDF, LiDAR, IMU, SLAM, etc.) in backend/app/data/technical_terms.json

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core translation infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create TranslatedContent model: translated_content table with chapter_id, lesson_id, target_language, original_content, translated_content, model_used, tokens_used, generation_time_ms, created_at, expires_at (NULL for indefinite) in backend/app/models/translated_content.py
- [ ] T007 Create TranslationRequest model: translation_requests table with user_id, chapter_id, lesson_id, target_language, cache_hit, response_time_ms, created_at in backend/app/models/translation_request.py
- [ ] T008 [P] Create Pydantic schemas for translation requests/responses (TranslateRequest, TranslateResponse) in backend/app/models/schemas.py
- [ ] T009 Create Alembic migration for translated_content, translation_requests tables with indexes
- [ ] T010 Run Alembic migration to create translation tables
- [ ] T011 [P] Load technical terms list into memory cache or database for fast lookup during translation

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Translation Service (Priority: P1) 🎯 MVP

**Goal**: Translate lesson content to Urdu using OpenAI GPT-4 with technical term preservation

**Independent Test**: POST /api/translate/chapter with {"chapter_id": "chapter-01", "lesson_id": "lesson-01", "target_language": "ur"} returns Urdu-translated Markdown content with preserved code blocks and technical terms

### Implementation for User Story 1

- [ ] T012 [P] [US1] Create Markdown processor: parse MDX, extract text nodes, preserve code blocks/components in backend/app/utils/markdown_processor.py (or reuse from embeddings/personalization)
- [ ] T013 [P] [US1] Implement MDX structure preservation: extract code blocks → replace with placeholders → translate → reinsert code in backend/app/services/markdown_processor.py
- [ ] T014 [P] [US1] Create translation prompt template for GPT-4: translate to Urdu while preserving technical terms, code, formulas in backend/app/utils/prompt_templates.py
- [ ] T015 [P] [US1] Implement technical term detection: scan content for technical terms from technical_terms.json in backend/app/utils/term_detector.py
- [ ] T016 [P] [US1] Implement technical term preservation: replace terms with placeholders before translation, reinsert with Urdu transliteration after in backend/app/services/term_preserver.py
- [ ] T017 [US1] Implement translation_service: call OpenAI GPT-4 with translation prompt, validate output in backend/app/services/translation_service.py
- [ ] T018 [US1] Add output validation: ensure headers preserved, code blocks unchanged, no broken MDX, technical terms in format "Term (اردو transliteration)"
- [ ] T019 [US1] Implement Urdu transliteration generation for technical terms (e.g., "ROS 2" → "آر او ایس ٹو")
- [ ] T020 [US1] Create cache_service: check cache by (chapter_id, lesson_id, target_language), store new translations in backend/app/services/translation_cache_service.py
- [ ] T021 [US1] Implement translation orchestration: orchestrate term detection → MDX parsing → translation → term reinsertion → cache store in backend/app/services/translation_orchestrator.py
- [ ] T022 [US1] Create POST /api/translate/chapter endpoint with chapter_id, lesson_id, target_language validation in backend/app/api/translation_routes.py
- [ ] T023 [US1] Implement translation flow: validate request → check cache → extract code/terms → translate (if cold) → reinsert code/terms → return translated content
- [ ] T024 [US1] Add error handling: OpenAI API failures (fallback to original content), MDX parsing errors, term preservation failures
- [ ] T025 [US1] Add token usage tracking and cost monitoring for translation requests

**Checkpoint**: At this point, translation should work (test with API: send lesson → receive Urdu translation with preserved code and technical terms)

---

## Phase 4: User Story 2 - Caching & Performance Optimization (Priority: P2) 🎯 MVP

**Goal**: Cache translated content indefinitely to minimize OpenAI costs (~$5-10 total for all 39 lessons)

**Independent Test**: First translation takes ~10-15 seconds (cold), subsequent requests for same lesson return < 500ms (warm)

### Implementation for User Story 2

- [ ] T026 [US2] Implement cache lookup: query translated_content by (chapter_id, lesson_id, target_language) in backend/app/services/translation_cache_service.py
- [ ] T027 [US2] Implement cache storage: insert translated content with indefinite expiration (expires_at = NULL)
- [ ] T028 [US2] Add cache hit/miss tracking: record in translation_requests table
- [ ] T029 [US2] Implement cache statistics endpoint: GET /api/translate/stats (cache hit rate, avg generation time, total translations, total cost) in backend/app/api/translation_routes.py
- [ ] T030 [US2] Optimize database queries: add composite index on (chapter_id, lesson_id, target_language)
- [ ] T031 [US2] Add cost tracking: calculate cumulative OpenAI costs for translation requests

**Checkpoint**: At this point, caching should work (test: translate same lesson twice → second request < 500ms, no OpenAI call)

---

## Phase 5: User Story 3 - Frontend Translation UI with RTL Support (Priority: P3) 🎯 MVP

**Goal**: Users see a "Translate to Urdu" button at the start of each chapter and can view Urdu content with proper RTL layout

**Independent Test**: Visit a chapter page → see "Translate to Urdu" button → click → loading indicator → see Urdu content with RTL layout → toggle back to English

### Implementation for User Story 3

- [ ] T032 [P] [US3] Create TranslateButton.tsx component with chapter_id, lesson_id props in book-source/src/components/Translation/TranslateButton.tsx
- [ ] T033 [P] [US3] Create TranslatedContent.tsx component to render Urdu Markdown with RTL layout in book-source/src/components/Translation/TranslatedContent.tsx
- [ ] T034 [P] [US3] Create LanguageToggle.tsx component to switch between English/Urdu views in book-source/src/components/Translation/LanguageToggle.tsx
- [ ] T035 [P] [US3] Create LoadingIndicator.tsx with progress message "Translating to Urdu..." in book-source/src/components/Translation/LoadingIndicator.tsx
- [ ] T036 [P] [US3] Create TypeScript interfaces: TranslateRequest, TranslateResponse in book-source/src/components/Translation/types.ts
- [ ] T037 [P] [US3] Create RTL CSS styles: .urdu-content { direction: rtl; text-align: right; } with LTR exceptions for code blocks in book-source/src/css/urdu-rtl.css
- [ ] T038 [P] [US3] Add Urdu font stack: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif in book-source/src/css/urdu-rtl.css
- [ ] T039 [US3] Implement translation API client: POST /api/translate/chapter with chapter_id, lesson_id, target_language="ur"
- [ ] T040 [US3] Implement state management: original content, translated content, currentLanguage flag ('en' | 'ur'), isLoading flag
- [ ] T041 [US3] Add TranslateButton to chapter header component (integrate into Docusaurus layout)
- [ ] T042 [US3] Implement translation flow: click button → call API → apply RTL CSS → render Urdu content
- [ ] T043 [US3] Implement language toggle functionality: switch between English and Urdu views (preserve both in state)
- [ ] T044 [US3] Add RTL layout logic: apply .urdu-content class when language='ur', remove when language='en'
- [ ] T045 [US3] Ensure code blocks remain LTR: override RTL direction for <code>, <pre> elements
- [ ] T046 [US3] Add error handling: display "Translation unavailable" message if API fails
- [ ] T047 [US3] Add success indicator: show "✓ اردو میں" (Urdu) badge when showing translated content

**Checkpoint**: At this point, translation UI with RTL should work (test: click button → see loading → see Urdu content with RTL → toggle back to English)

---

## Phase 6: User Story 4 - Advanced Translation Features (Priority: P4)

**Goal**: Add status check, preview, and cache management endpoints

**Independent Test**: GET /api/translate/status/{chapter_id}/{lesson_id} returns cache status; POST /api/translate/preview returns translated content without caching

### Implementation for User Story 4

- [ ] T048 [P] [US4] Create GET /api/translate/status/{chapter_id}/{lesson_id} endpoint: check if Urdu translation exists in cache in backend/app/api/translation_routes.py
- [ ] T049 [P] [US4] Create POST /api/translate/preview endpoint: translate content without caching (for testing/validation) in backend/app/api/translation_routes.py
- [ ] T050 [US4] Create DELETE /api/translate/cache/clear endpoint (admin): clear cache for specific chapter or all chapters
- [ ] T051 [US4] Implement cache invalidation logic: delete by chapter_id or clear all Urdu translations
- [ ] T052 [US4] Add translation history tracking: store which lessons user has translated (in user_profiles table or separate table)
- [ ] T053 [US4] Frontend: Add "Preview Translation" mode (call preview endpoint instead of regular endpoint)
- [ ] T054 [US4] Frontend: Show cache status indicator: "Cached" or "Generating..." before translation
- [ ] T055 [US4] Add auto-translate feature: automatically translate to Urdu if user's preferred_language='ur' in Better Auth profile

**Checkpoint**: At this point, advanced features should work (test: check status, preview, clear cache, auto-translate)

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T056 [P] Add comprehensive docstrings (Google style) to all translation services
- [ ] T057 [P] Add OpenAPI documentation tags to all translation endpoints
- [ ] T058 [P] Implement request logging for translation operations (chapter_id, lesson_id, generation time, cache hit/miss, tokens used)
- [ ] T059 [P] Add input sanitization for translation requests (prevent prompt injection)
- [ ] T060 [P] Optimize OpenAI prompts: reduce token usage while maintaining translation quality
- [ ] T061 [P] Add translation quality metrics: track user feedback on translation quality (optional)
- [ ] T062 [P] Implement user feedback mechanism: "Was this translation helpful?" thumbs up/down
- [ ] T063 [P] Frontend: Add animation for language transition (fade in/out)
- [ ] T064 [P] Frontend: Add mobile responsive styles for translation UI
- [ ] T065 [P] Frontend: Add keyboard shortcuts (U for Urdu, E for English)
- [ ] T066 [P] Frontend: Add Urdu typography enhancements (line height, letter spacing adjustments)
- [ ] T067 [P] Create README.md with translation setup instructions in backend/README.md
- [ ] T068 [P] Add environment variables documentation for translation settings
- [ ] T069 Run full integration test: translate lesson → verify Urdu content → verify RTL layout → verify code blocks LTR → toggle English → clear cache

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - **US1 (Translation)**: Can start after Foundational
  - **US2 (Caching)**: Depends on US1 (requires translation service)
  - **US3 (Frontend UI + RTL)**: Depends on US1 (requires API endpoints)
  - **US4 (Advanced Features)**: Depends on US2 (extends caching logic)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (Translation)**: Can start after Foundational - No dependencies on other stories
- **US2 (Caching)**: Depends on US1 completion (requires translation_service)
- **US3 (Frontend UI + RTL)**: Depends on US1 completion (requires POST /api/translate/chapter endpoint)
- **US4 (Advanced Features)**: Depends on US2 completion (extends caching)

### Within Each User Story

- **US1**: Technical terms list → Markdown processor → term detection → translation prompt → translation service → cache service → API endpoint
- **US2**: Cache lookup → cache storage → cache tracking → cache stats → cost tracking
- **US3**: React components → RTL CSS → API client → state management → integration into chapter pages → RTL layout logic
- **US4**: Status endpoint → preview endpoint → cache clear endpoint → auto-translate → frontend features

### Parallel Opportunities

- **Phase 1**: T003 (OpenAI config), T004 (frontend packages), T005 (technical terms list) can run in parallel
- **Phase 2**: T008 (schemas), T011 (load technical terms) can run in parallel after T006-T007 complete
- **US1**: T012-T013 (Markdown processor), T014 (translation prompt), T015-T016 (term detection/preservation) can run in parallel
- **US2**: T026-T027 (cache operations) can run in parallel
- **US3**: T032-T038 (all React components, types, and CSS) can run in parallel
- **US4**: T048-T049 (status and preview endpoints) can run in parallel
- **Phase 7**: Most polish tasks can run in parallel (T056-T068)

---

## Parallel Example: User Story 3 (Frontend UI + RTL)

```bash
# Launch all frontend components in parallel:
Task: "Create TranslateButton.tsx in book-source/src/components/Translation/TranslateButton.tsx"
Task: "Create TranslatedContent.tsx in book-source/src/components/Translation/TranslatedContent.tsx"
Task: "Create LanguageToggle.tsx in book-source/src/components/Translation/LanguageToggle.tsx"
Task: "Create LoadingIndicator.tsx in book-source/src/components/Translation/LoadingIndicator.tsx"
Task: "Create TypeScript interfaces in book-source/src/components/Translation/types.ts"
Task: "Create RTL CSS styles in book-source/src/css/urdu-rtl.css"
Task: "Add Urdu font stack in book-source/src/css/urdu-rtl.css"

# After all complete, integrate them:
Task: "Implement translation API client and state management"
Task: "Add TranslateButton to chapter header component"
Task: "Implement RTL layout logic and code block LTR overrides"
```

---

## Implementation Strategy

### MVP First (US1 + US2 + US3 Only)

**Goal**: Get Urdu translation working with indefinite caching and RTL UI

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 (Translation) → Test with API (Postman/curl)
4. Complete Phase 4: US2 (Caching) → Verify indefinite cache, no duplicate OpenAI calls
5. Complete Phase 5: US3 (Frontend UI + RTL) → Test in browser with RTL layout
6. **STOP and VALIDATE**: Test end-to-end translation flow with RTL
7. Deploy/demo if ready

**MVP Scope**: US1 (translate) + US2 (cache) + US3 (UI + RTL) = **Working Urdu translation** ✅

### Full Feature Set

After MVP validation:

8. Complete Phase 6: US4 (Advanced Features)
9. Complete Phase 7: Polish & Cross-Cutting Concerns
10. Final integration testing with Better Auth (auto-translate for Urdu users)

### Incremental Delivery Milestones

- **Milestone 1**: Setup + Foundational → Infrastructure ready
- **Milestone 2**: + US1 → Translation works (test with API)
- **Milestone 3**: + US2 → Caching works (100% hit rate after first translation)
- **Milestone 4**: + US3 → **MVP: Urdu translation UI with RTL functional** 🎯
- **Milestone 5**: + US4 → Advanced features (status, preview, auto-translate)
- **Milestone 6**: + Polish → **Production-ready**

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Focus on MVP first (US1 + US2 + US3) before adding advanced features
- Can integrate with Better Auth for `preferred_language` auto-translation
- **Most cost-effective feature**: Indefinite caching means each lesson translated once (~$5-10 total for all 39 lessons)
- RTL CSS critical for Urdu readability - must override for code blocks to keep them LTR
- Technical term preservation ensures clarity (e.g., "ROS 2 (آر او ایس ٹو)")

---

## Total Task Count

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 6 tasks
- **Phase 3 (US1 - Translation)**: 14 tasks
- **Phase 4 (US2 - Caching)**: 6 tasks
- **Phase 5 (US3 - Frontend UI + RTL)**: 16 tasks
- **Phase 6 (US4 - Advanced Features)**: 8 tasks
- **Phase 7 (Polish)**: 14 tasks

**Total**: 69 tasks

**MVP Tasks** (US1 + US2 + US3): 5 (setup) + 6 (foundational) + 14 (US1) + 6 (US2) + 16 (US3) = **47 tasks**

**Parallel Opportunities**: 30+ tasks can run in parallel at various stages
