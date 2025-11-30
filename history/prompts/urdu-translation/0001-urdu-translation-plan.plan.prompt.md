---
id: 0001
title: Urdu Translation Implementation Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: urdu-translation
branch: main
user: system
command: /sp.plan
labels: ["urdu-translation", "i18n", "rtl", "openai", "gpt-4", "caching", "localization", "phase-2", "bonus-50-points"]
links:
  spec: null
  ticket: null
  adr: specs/urdu-translation/plan.md#architectural-decisions
  pr: null
files:
  - specs/urdu-translation/plan.md
tests:
  - None (planning phase)
---

## Prompt

Create implementation plan for Urdu Translation (Bonus - 50 points).

**Context**: Physical AI textbook backend (FastAPI, OpenAI, Neon Postgres)

**Requirements**:
- Translate chapter content to Urdu on-demand
- Button at start of each chapter
- Cache translations
- Support RTL layout

**Constitution**: Follow .specify/memory/constitution.md (Backend Development Principles)

**Output**: Detailed plan with phases (Translation Service, API Endpoints, Frontend Integration, Caching, RTL Support). Include dependencies, estimated time.

## Response snapshot

Created comprehensive implementation plan (`specs/urdu-translation/plan.md`) for Urdu Translation following constitution-mandated backend development workflow.

**Key Outputs**:

1. **Technical Context** - Defined stack (Python 3.11+, FastAPI, OpenAI GPT-4, Neon Postgres, React/TypeScript, RTL CSS)
2. **Constitution Check** - All gates PASS (Backend Development Principles, Educational Excellence)
3. **Project Structure** - Defined backend/app/{models,api,services,utils} and book-source/src/components/Translation/ + RTL CSS
4. **Phase 0: Research** - 7 research tasks (LLM translation for technical content, RTL layout best practices, technical term preservation, MDX preservation, caching, cost optimization, code/formula handling)
5. **Phase 1: Design** - Data models (translated_content cache, translation_requests tracking), API contracts (4 endpoints), quickstart guide

6. **Architectural Decisions** - 6 ADRs documented:
   - **ADR-001**: GPT-4-turbo-preview (vs GPT-3.5, vs Google Translate, vs DeepL)
   - **ADR-002**: Indefinite caching (vs time-based expiration)
   - **ADR-003**: Technical term preservation (English + Urdu transliteration)
   - **ADR-004**: CSS `direction: rtl` for Urdu containers, LTR for code
   - **ADR-005**: Opt-in "Translate to Urdu" button
   - **ADR-006**: Extract code/formulas before translation, reinsert after

7. **Database Schema**:
   - **translated_content**: id, chapter_id, lesson_id, target_language, original_content, translated_content, model_used, tokens_used, generation_time_ms, created_at, expires_at (NULL = indefinite)
   - **translation_requests**: id, user_id, chapter_id, lesson_id, target_language, cache_hit, response_time_ms, created_at
   - **Technical Terms List**: ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, Python, URDF, LiDAR, IMU, SLAM, etc.

8. **API Endpoints Designed**:
   - POST /api/translate/chapter (generate Urdu translation)
   - GET /api/translate/status/{chapter_id}/{lesson_id} (check cache)
   - POST /api/translate/preview (test without caching)
   - DELETE /api/translate/cache/clear (admin)

9. **LLM Prompt Template**:
   - Preserve structure/code/formulas, translate text content
   - Technical term preservation with Urdu transliteration
   - Example: "ROS 2 (آر او ایس ٹو)"
   - Formal Urdu suitable for technical education

10. **RTL CSS Implementation**:
    - `.urdu-content { direction: rtl; text-align: right; }`
    - Preserve LTR for code blocks, formulas, interactive components
    - Urdu font stack: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif
    - Language toggle button (sticky, top-right)

11. **Frontend Components**:
    - TranslateButton.tsx ("Translate to Urdu" button)
    - TranslatedContent.tsx (display Urdu with RTL)
    - LanguageToggle.tsx (switch English/Urdu)
    - LoadingIndicator.tsx (during translation)

12. **Cost Estimation** (Most Cost-Effective Feature):
    - Cold cache: $5.38 (39 lessons × $0.138/lesson) - **One-time cost**
    - Warm cache: $0 (indefinite caching, 90%+ hit rate)
    - Total hackathon estimate: ~$5-10 (including testing)
    - **Lowest cost feature** due to single-variant translations + indefinite caching

13. **Success Criteria**:
    - Functional: Button works, Urdu displays with RTL, terms preserved, code stays LTR, toggle works, caching works
    - Performance: Cold < 15s, Warm < 500ms, Cache hit > 90%, Tokens < 6000
    - Quality: Accurate translation, terms preserved, no broken MDX, RTL renders correctly, 70% test coverage
    - RTL: Urdu right-aligned, code left-aligned, UI not affected, fonts render correctly

14. **Integration Points**:
    - Better Auth: `preferred_language` field, auto-translate for Urdu users
    - Content Personalization: Personalize → Translate (layered caching)
    - RAG Chatbot: Urdu chat queries, Urdu content retrieval

15. **Timeline**: 12-16 hours (1.5-2 days)

**Next Steps**:
1. Run `/sp.tasks` to generate tasks.md
2. Execute Phase 0 research (RTL patterns, technical term preservation, Urdu typography)
3. Execute Phase 1 design (data-model.md, contracts/translation-api-spec.yaml, quickstart.md)
4. Begin implementation via `/sp.implement`

## Outcome

- ✅ Impact: Comprehensive plan created for 50-point bonus hackathon requirement (Urdu Translation)
- 🧪 Tests: None (planning phase - tests defined in plan)
- 📁 Files: specs/urdu-translation/plan.md (850+ lines)
- 🔁 Next prompts: `/sp.tasks` → Phase 0 research → Phase 1 design → `/sp.implement`
- 🧠 Reflection: Plan follows constitution Backend Development Principles + Educational Excellence. GPT-4 for high-quality technical translation. Indefinite caching makes this the **most cost-effective feature** (~$5-10 total). RTL CSS with selective LTR for code/formulas ensures proper rendering. Technical term preservation (English + Urdu transliteration) maintains clarity. Extract-translate-reinsert pattern protects code/formulas from LLM modification. Realistic timeline (12-16 hours). Clear integration with Better Auth (language preference) and future RAG Chatbot (Urdu queries).

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Execute `/sp.tasks` to validate plan completeness by generating actionable task breakdown
