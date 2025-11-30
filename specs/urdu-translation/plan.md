# Implementation Plan: Urdu Translation

**Branch**: `main` | **Date**: 2025-11-29 | **Feature**: Urdu Translation (Bonus - 50 points)
**Input**: Hackathon requirements for on-demand Urdu translation with RTL support

## Summary

Build an on-demand Urdu translation system for the Physical AI textbook that enables Urdu-speaking students to access content in their native language. The system will:
- Translate chapter content to Urdu on-demand using OpenAI
- Provide a "Translate to Urdu" button at the start of each chapter
- Cache translations to reduce API costs and improve performance
- Support RTL (Right-to-Left) layout for Urdu text rendering
- Preserve technical terms in English (transliterated) while translating explanations
- Maintain Markdown/MDX structure (code blocks, components, formulas)

**Technical Approach**: FastAPI backend with OpenAI GPT-4 for high-quality technical translation, Neon Postgres for caching translated versions, React component with RTL CSS support in Docusaurus.

## Technical Context

**Language/Version**: Python 3.11+ (FastAPI backend), TypeScript/React 18 (Docusaurus frontend)
**Primary Dependencies**: FastAPI, OpenAI Python SDK, SQLAlchemy, Pydantic, React, CSS RTL support (direction: rtl)
**Storage**: Neon Postgres (translated_content cache, translation_requests tracking)
**Testing**: pytest (backend unit/integration tests), Jest (frontend component tests)
**Target Platform**: Cloud-deployed API (Vercel/Railway/Render) + Static Docusaurus site (GitHub Pages/Netlify)
**Project Type**: Web (backend API + frontend component)
**Performance Goals**:
- First translation (cold): < 15 seconds (LLM generation)
- Cached translation (warm): < 500ms (database retrieval)
- Cache hit rate: > 90% (Urdu translations reused across users)
**Constraints**:
- Must preserve Markdown/MDX structure (headers, code blocks, components)
- Must preserve English technical terms (ROS 2, Gazebo, NVIDIA Isaac, etc.)
- Must preserve code blocks exactly (no translation)
- Must preserve mathematical formulas (LaTeX)
- RTL layout must not break existing LTR components (code blocks, diagrams)
**Scale/Scope**:
- 39 lessons to translate
- ~1 translation variant per lesson (Urdu is single target language)
- 100s of Urdu-speaking users (hackathon scale)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Phase-Driven Architecture**: This is Phase 2 (RAG + Features) - Urdu translation is bonus feature
✅ **Spec-Driven Development**: Following `/sp.plan` → `/sp.implement` workflow (backend development path)
✅ **Backend Development Principles**: Using FastAPI, Neon Postgres, proper error handling, security standards
✅ **Code Quality Standards**: Python 3.11+ with type hints, comprehensive docstrings, test coverage > 70%
✅ **API Design Standards**: RESTful conventions, consistent response formats, proper HTTP status codes
✅ **Security Requirements**: Environment variables for secrets, rate limiting (protect OpenAI API), input validation
✅ **File Organization**: Backend follows `backend/app/{main,config,models,api,services,utils}` structure
✅ **Educational Excellence**: Translation must maintain technical accuracy, preserve pedagogical structure

**No violations detected. All gates PASS.**

## Project Structure

### Documentation (this feature)

```text
specs/urdu-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0: Translation best practices, RTL layout patterns
├── data-model.md        # Phase 1: Database schemas (translated_content, requests)
├── quickstart.md        # Phase 1: Setup instructions for translation service
├── contracts/           # Phase 1: API contracts (translation endpoints)
│   └── translation-api-spec.yaml
└── tasks.md             # Phase 2: Created by /sp.tasks command
```

### Source Code (repository root)

```text
backend/                          # FastAPI backend
├── app/
│   ├── main.py                  # FastAPI app (add translation routes)
│   ├── config.py                # Add OpenAI config for translation
│   ├── models/
│   │   ├── translated_content.py  # SQLAlchemy model for cache
│   │   └── translation_request.py # SQLAlchemy model for tracking
│   ├── api/
│   │   └── translation_routes.py # /api/translate endpoints
│   ├── services/
│   │   ├── translation_service.py # Core translation logic
│   │   ├── urdu_translator.py   # LLM-based Urdu translation
│   │   ├── cache_service.py     # Caching strategy (chapter-based)
│   │   └── markdown_processor.py # Preserve MDX structure during translation
│   └── utils/
│       ├── technical_terms.py   # List of terms to preserve in English
│       └── prompt_templates.py  # LLM prompts for Urdu translation
├── tests/
│   ├── unit/
│   │   ├── test_translation_service.py
│   │   ├── test_urdu_translator.py
│   │   └── test_cache_service.py
│   └── integration/
│       └── test_translation_api.py
└── alembic/
    └── versions/
        └── 0004_create_translation_tables.py

book-source/                      # Existing Docusaurus site
├── src/
│   ├── components/
│   │   ├── Translation/
│   │   │   ├── TranslateButton.tsx  # "Translate to Urdu" button
│   │   │   ├── TranslatedContent.tsx # Display Urdu version with RTL
│   │   │   ├── LanguageToggle.tsx  # Switch between English/Urdu
│   │   │   ├── LoadingIndicator.tsx # Loading state during translation
│   │   │   └── types.ts              # TypeScript interfaces
│   │   └── ChapterHeader.tsx         # Modified to include TranslateButton
│   └── css/
│       └── rtl-support.css       # RTL layout styles for Urdu
└── docusaurus.config.ts          # Add Urdu locale configuration (optional)
```

**Structure Decision**: Extend existing backend with translation service. Frontend components integrate into chapter pages with RTL CSS support. Reuse caching infrastructure from content personalization.

## Complexity Tracking

> No violations detected - this section is empty.

---

## Phase 0: Research & Technology Decisions

**Prerequisites**: Constitution approved
**Output**: `research.md` with all technology choices documented

### Research Tasks

1. **LLM Translation Strategies for Technical Content**
   - Research: Few-shot prompting for technical translation, glossary injection
   - Decision needed: Translation prompt structure, technical term handling, output format
   - Reference: OpenAI best practices for translation, technical translation guidelines

2. **Urdu Language and RTL Layout Best Practices**
   - Research: RTL CSS patterns, bidirectional text (bidi) handling, font selection for Urdu
   - Decision needed: RTL CSS approach (direction: rtl vs logical properties), font stack
   - Reference: W3C RTL guidelines, Unicode bidirectional algorithm

3. **Technical Term Preservation**
   - Research: How to preserve English technical terms in Urdu translations
   - Decision needed: Which terms to preserve (ROS 2, Gazebo, Isaac, Python, etc.), transliteration strategy
   - Reference: Technical translation standards, localization best practices

4. **Markdown/MDX Preservation in Translation**
   - Research: How to preserve structure while translating text (headers, code blocks, components)
   - Decision needed: Parsing strategy (AST-based vs regex), component handling
   - Reference: Markdown parsers, MDX specification

5. **Caching Strategy for Translations**
   - Research: Cache key generation (chapter-based), cache invalidation strategies
   - Decision needed: Cache TTL (indefinite vs time-based), storage format
   - Reference: Caching best practices, translation memory systems

6. **OpenAI Cost Optimization for Translation**
   - Research: Token usage minimization, batch processing, model selection (GPT-4 vs GPT-3.5)
   - Decision needed: Which model to use, token limits per translation, fallback strategies
   - Reference: OpenAI pricing, translation cost benchmarks

7. **Code Block and Formula Handling**
   - Research: How to exclude code blocks and LaTeX formulas from translation
   - Decision needed: Extraction strategy, placeholder replacement, reassembly
   - Reference: Translation tool best practices, code preservation patterns

### Research Outputs

All findings documented in `specs/urdu-translation/research.md` with format:
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
**Output**: `data-model.md`, `contracts/translation-api-spec.yaml`, `quickstart.md`

### 1.1 Data Model Design (`data-model.md`)

**Postgres Schema (Neon)**:

```sql
-- Translated content cache (stores Urdu translations)
CREATE TABLE translated_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(50) NOT NULL,  -- e.g., "chapter-01"
    lesson_id VARCHAR(50) NOT NULL,   -- e.g., "lesson-01-introduction"
    target_language VARCHAR(10) NOT NULL,  -- e.g., "ur" (Urdu), "ar" (Arabic - future)
    original_content TEXT NOT NULL,   -- Original English Markdown content
    translated_content TEXT NOT NULL,  -- Translated Urdu Markdown content
    model_used VARCHAR(50),  -- e.g., "gpt-4-turbo-preview"
    tokens_used INTEGER,  -- OpenAI token count
    generation_time_ms INTEGER,  -- Time to generate (for analytics)
    created_at TIMESTAMP DEFAULT NOW(),
    expires_at TIMESTAMP,  -- Cache TTL (NULL = indefinite)
    UNIQUE(chapter_id, lesson_id, target_language)
);

-- Translation requests (tracking and analytics)
CREATE TABLE translation_requests (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),  -- Optional: link to authenticated user
    chapter_id VARCHAR(50) NOT NULL,
    lesson_id VARCHAR(50) NOT NULL,
    target_language VARCHAR(10) NOT NULL,
    cache_hit BOOLEAN DEFAULT FALSE,  -- Was content served from cache?
    response_time_ms INTEGER,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Indexes
CREATE INDEX idx_translated_content_lookup ON translated_content(chapter_id, lesson_id, target_language);
CREATE INDEX idx_translated_content_expires ON translated_content(expires_at);
CREATE INDEX idx_translation_requests_user ON translation_requests(user_id);
CREATE INDEX idx_translation_requests_chapter ON translation_requests(chapter_id);
CREATE INDEX idx_translation_requests_language ON translation_requests(target_language);
```

**Technical Terms List** (to preserve in English):

```python
TECHNICAL_TERMS = [
    # Frameworks and Tools
    "ROS 2", "Gazebo", "Unity", "NVIDIA Isaac", "Isaac Sim", "Isaac ROS",
    "Nav2", "OpenAI", "Whisper", "GPT", "LLM", "VLA",

    # Programming Languages and Concepts
    "Python", "rclpy", "URDF", "SDF", "Xacro", "YAML", "JSON",

    # Robotics Concepts (transliterate in Urdu)
    "node", "topic", "service", "action", "publisher", "subscriber",
    "LiDAR", "IMU", "SLAM", "VSLAM", "sensor fusion",

    # AI/ML Terms
    "reinforcement learning", "policy", "reward", "actor-critic",
    "neural network", "deep learning", "transformer",

    # File Extensions and Commands
    ".py", ".md", ".yaml", "rosrun", "rosbag", "colcon build",

    # URLs and Code
    # All code blocks, URLs, and file paths preserved exactly
]
```

### 1.2 API Contracts (`contracts/translation-api-spec.yaml`)

**Endpoints**:

1. **POST /api/translate/chapter** - Translate a chapter to Urdu
   ```yaml
   /api/translate/chapter:
     post:
       summary: Generate Urdu translation of a chapter
       requestBody:
         content:
           application/json:
             schema:
               type: object
               required: [chapter_id, lesson_id, target_language]
               properties:
                 chapter_id:
                   type: string
                   example: "chapter-01"
                 lesson_id:
                   type: string
                   example: "lesson-01-introduction"
                 target_language:
                   type: string
                   enum: [ur]  # Urdu (future: ar, hi, etc.)
                   default: ur
       responses:
         200:
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
                       translated_content:
                         type: string
                         description: Translated Urdu Markdown content
                       cache_hit:
                         type: boolean
                       metadata:
                         type: object
                         properties:
                           model_used: string
                           tokens_used: integer
                           generation_time_ms: integer
   ```

2. **GET /api/translate/status/{chapter_id}/{lesson_id}** - Check if translation exists
3. **POST /api/translate/preview** - Preview translation without caching (for testing)
4. **DELETE /api/translate/cache/clear** - Clear cache for a language (admin endpoint)

Full OpenAPI spec generated in `contracts/translation-api-spec.yaml`.

### 1.3 Quickstart Guide (`quickstart.md`)

Step-by-step developer setup:
1. Configure OpenAI API key in environment variables
2. Run database migrations (translated_content, translation_requests tables)
3. Test LLM translation prompts with sample Urdu output
4. Implement caching logic with chapter-based keys
5. Add TranslateButton component to chapter pages
6. Add RTL CSS support for Urdu text rendering
7. Test translation flow (button click → API call → display Urdu content with RTL)

### 1.4 Agent Context Update

Run constitution-mandated agent context update:
```powershell
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

Adds Urdu Translation technologies to agent context:
- OpenAI GPT-4 for translation
- RTL CSS patterns
- Urdu font rendering

---

## Phase 2: Task Breakdown

**Note**: This phase is executed by `/sp.tasks` command, NOT by `/sp.plan`.

The `/sp.tasks` command will generate `specs/urdu-translation/tasks.md` with detailed implementation tasks based on this plan.

**Expected Task Structure**:
1. Database Schema & Migrations (translated_content, translation_requests tables)
2. Technical Terms List (create comprehensive list of terms to preserve)
3. Markdown/MDX Processor (preserve structure, extract text nodes, exclude code/formulas)
4. LLM Translation Prompts (few-shot prompts with technical term preservation)
5. Urdu Translator Service (OpenAI integration, output validation)
6. Caching Service (retrieve/store translations, indefinite TTL)
7. Translation API Endpoints (POST /api/translate/chapter, etc.)
8. Frontend TranslateButton Component (trigger translation)
9. Frontend TranslatedContent Component (display Urdu with RTL, toggle English/Urdu)
10. RTL CSS Styling (direction: rtl, font selection, layout adjustments)
11. Testing (Unit tests for translator, integration tests for API, RTL rendering tests)
12. Cost Monitoring (track OpenAI token usage, optimize prompts)

---

## Phase 3-7: Implementation Phases

**Note**: These phases are executed by `/sp.implement` command, NOT by `/sp.plan`.

The `/sp.implement` command will execute tasks from `tasks.md` in dependency order, implementing:
- Phase 3: Database & Caching Infrastructure
- Phase 4: LLM Integration & Prompt Engineering
- Phase 5: Translation Service
- Phase 6: API Endpoints
- Phase 7: Frontend Components (Translation UI + RTL Support)
- Phase 8: Testing & Validation
- Phase 9: Cost Optimization & Monitoring

---

## Architectural Decisions

### ADR-001: GPT-4 vs GPT-3.5 for Urdu Translation
**Decision**: Use GPT-4-turbo-preview for Urdu translation
**Rationale**: Superior quality for technical translation, better preservation of technical terms, nuanced understanding of context. Cost justified by indefinite caching (each lesson translated once).
**Alternatives**:
- GPT-3.5-turbo (rejected: lower quality for technical Urdu, may mistranslate technical terms)
- Google Translate API (rejected: poor quality for technical content, no context awareness)
- DeepL (rejected: limited Urdu support)

### ADR-002: Caching Strategy - Chapter-Based with Indefinite TTL
**Decision**: Cache translations indefinitely (expires_at = NULL), invalidate manually when lessons updated
**Rationale**: Urdu translations are language-agnostic (no user-specific variations), so all Urdu users share the same translation. High cache hit rate (90%+) drastically reduces costs.
**Alternatives**:
- Time-based expiration (rejected: unnecessary re-translations, higher costs)
- Per-user caching (rejected: identical translations duplicated, storage waste)

### ADR-003: Technical Term Preservation
**Decision**: Preserve English technical terms with Urdu transliteration in parentheses
**Example**:
- "ROS 2" → "ROS 2 (آر او ایس ٹو)"
- "Gazebo" → "Gazebo (گزیبو)"
- "node" → "node (نوڈ)"

**Rationale**: Technical terms are universally recognized in English. Transliteration provides pronunciation guide while maintaining clarity.
**Alternatives**:
- Full translation (rejected: creates confusion, loses international context)
- English-only (rejected: less accessible for Urdu readers)

### ADR-004: RTL Layout Strategy
**Decision**: Use CSS `direction: rtl` for Urdu content containers, preserve `direction: ltr` for code blocks
**Rationale**: Modern CSS supports RTL layouts natively. Selective application ensures code blocks remain LTR (readable).
**CSS Implementation**:
```css
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif;
}

.urdu-content code,
.urdu-content pre {
  direction: ltr;  /* Preserve LTR for code */
  text-align: left;
}
```

**Alternatives**:
- Full page RTL (rejected: breaks navigation, UI components)
- Manual positioning (rejected: fragile, hard to maintain)

### ADR-005: Translation Trigger
**Decision**: Opt-in "Translate to Urdu" button at chapter start
**Rationale**: User consent, preserves English as default (international audience), clear UX.
**Alternatives**:
- Automatic detection (rejected: no user control, may incorrectly assume language preference)
- Browser language detection (rejected: users may prefer English for technical content)

### ADR-006: Code Block and Formula Handling
**Decision**: Extract code blocks and LaTeX formulas before translation, reinsert after translation
**Rationale**: Ensures code and formulas remain unchanged. Placeholders prevent LLM from attempting translation.
**Process**:
1. Parse Markdown AST
2. Extract code blocks → replace with `<<CODE_BLOCK_1>>`, `<<CODE_BLOCK_2>>`, etc.
3. Extract LaTeX formulas → replace with `<<FORMULA_1>>`, `<<FORMULA_2>>`, etc.
4. Translate text content
5. Reinsert code blocks and formulas

**Alternatives**:
- Trust LLM to preserve code (rejected: unreliable, may alter syntax)
- Post-translation validation (rejected: expensive, error-prone)

---

## LLM Prompt Design

### Base Translation Prompt

```python
URDU_TRANSLATION_PROMPT = """
You are an expert technical translator specializing in English to Urdu translation for educational robotics content.

**Original Lesson Content (English):**
{original_content}

**Task:**
Translate the lesson content to Urdu while following these rules:

1. **Preserve Structure:**
   - Keep all headers (# ## ###) exactly as they are
   - Keep all code blocks unchanged (marked as <<CODE_BLOCK_N>>)
   - Keep all formulas unchanged (marked as <<FORMULA_N>>)
   - Keep all Markdown components (:::note, :::tip, <InteractivePython>, etc.)

2. **Technical Terms:**
   - Preserve these terms in English: ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, Python, URDF, LiDAR, IMU, SLAM
   - Add Urdu transliteration in parentheses after first mention
   - Example: "ROS 2 (آر او ایس ٹو)"

3. **Translation Quality:**
   - Use formal Urdu (suitable for technical education)
   - Maintain technical accuracy
   - Preserve pedagogical structure
   - Use clear, concise language

4. **Examples and Analogies:**
   - Translate examples to Urdu, but keep code snippets in English
   - Adapt analogies to be culturally appropriate for Urdu speakers

**Output Format:**
Return ONLY the translated Urdu Markdown content. Do not add explanations or comments.

**Example Translation:**

English:
"ROS 2 is a framework for robot software development. It provides communication between nodes using topics."

Urdu:
"ROS 2 (آر او ایس ٹو) روبوٹ سافٹ ویئر ڈیولپمنٹ کے لیے ایک فریم ورک ہے۔ یہ topics (ٹاپکس) کے ذریعے nodes (نوڈز) کے درمیان رابطہ فراہم کرتا ہے۔"

Now translate the content:
"""
```

---

## RTL CSS Implementation

### Urdu-Specific Styles

```css
/* book-source/src/css/rtl-support.css */

/* Urdu content container */
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", "Alvi Nastaleeq", serif;
  line-height: 2;  /* Urdu requires more line spacing */
}

/* Preserve LTR for code blocks */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
  font-family: "Fira Code", "Consolas", "Monaco", monospace;
}

/* Preserve LTR for numbers and formulas */
.urdu-content .math,
.urdu-content .katex {
  direction: ltr;
}

/* Flip icons and UI elements */
.urdu-content .arrow-right::before {
  content: "←";  /* Flip arrow direction */
}

.urdu-content .arrow-left::before {
  content: "→";
}

/* Adjust list markers for RTL */
.urdu-content ul {
  list-style-position: inside;
  padding-right: 1rem;
  padding-left: 0;
}

.urdu-content ol {
  padding-right: 1rem;
  padding-left: 0;
}

/* Headers remain left-aligned for consistency */
.urdu-content h1,
.urdu-content h2,
.urdu-content h3 {
  text-align: right;
}

/* Interactive components remain LTR */
.urdu-content .interactive-python,
.urdu-content .quiz,
.urdu-content .try-with-ai {
  direction: ltr;
  text-align: left;
}

/* Language toggle button */
.language-toggle {
  position: sticky;
  top: 80px;
  right: 20px;
  z-index: 100;
  background: var(--ifm-color-primary);
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.3s ease;
}

.language-toggle:hover {
  background: var(--ifm-color-primary-dark);
}
```

### Font Loading

```typescript
// book-source/docusaurus.config.ts - Add to headTags

{
  tagName: "link",
  attributes: {
    rel: "preconnect",
    href: "https://fonts.googleapis.com",
  },
},
{
  tagName: "link",
  attributes: {
    rel: "stylesheet",
    href: "https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap",
  },
},
```

---

## Dependencies

### External Services
- **Neon Postgres**: Free tier (3GB storage, shared compute) - already provisioned
- **OpenAI API**: Pay-as-you-go (GPT-4 completions)

### Python Packages (backend/requirements.txt - already included)
```
# No new dependencies required (reuse existing OpenAI, SQLAlchemy, FastAPI)
```

### Frontend Packages (book-source/package.json - already included)
```json
{
  "dependencies": {
    "react-markdown": "^9.0.1"  // Already added for content personalization
  }
}
```

---

## Success Criteria

✅ **Functional**:
- [ ] Users can click "Translate to Urdu" button on any chapter
- [ ] Urdu translation displays with proper RTL layout
- [ ] Technical terms preserved in English with Urdu transliteration
- [ ] Code blocks remain in English with LTR layout
- [ ] Users can toggle between English and Urdu
- [ ] Translations cached (subsequent requests < 500ms)

✅ **Performance**:
- [ ] First translation (cold): < 15 seconds
- [ ] Cached translation (warm): < 500ms
- [ ] Cache hit rate: > 90%
- [ ] OpenAI token usage: < 6000 tokens per translation

✅ **Quality**:
- [ ] Urdu translation is technically accurate
- [ ] Technical terms correctly preserved
- [ ] No broken Markdown/MDX (headers, code blocks, components)
- [ ] RTL layout renders correctly on all browsers
- [ ] Test coverage > 70% (translator, caching)

✅ **RTL Support**:
- [ ] Urdu text right-aligned
- [ ] Code blocks remain left-aligned
- [ ] Navigation and UI components not affected
- [ ] Fonts render correctly (Nastaliq script)

✅ **Cost Optimization**:
- [ ] Caching reduces OpenAI API costs by > 90%
- [ ] Token usage monitored and logged
- [ ] Fallback to English if translation fails

---

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| LLM mistranslation of technical terms | Medium | High | Technical terms list, validation rules, manual review |
| High OpenAI API costs | Low | Medium | Indefinite caching (90%+ hit rate), token usage monitoring |
| RTL layout breaking LTR components | Medium | Medium | Selective RTL application, thorough testing |
| Translation quality inconsistency | Medium | Medium | Few-shot prompting, native speaker review |
| Font rendering issues | Low | Low | Multiple font fallbacks, Google Fonts CDN |
| Cache storage growth | Low | Low | Indefinite TTL acceptable (39 lessons × ~10KB = ~400KB total) |

---

## Cost Estimation

### OpenAI API Costs (GPT-4-turbo-preview)

**Assumptions**:
- 39 lessons to translate
- 1 Urdu translation per lesson (indefinite cache)
- Average lesson length: 3000 tokens
- GPT-4-turbo pricing: $0.01/1K input tokens, $0.03/1K output tokens

**Cold Cache (First Generation - One-Time Cost)**:
- Input tokens per lesson: 3000 tokens (lesson content) + 300 tokens (prompt) = 3300 tokens
- Output tokens per lesson: ~3500 tokens (Urdu translation)
- Cost per lesson: (3.3K × $0.01) + (3.5K × $0.03) = $0.033 + $0.105 = **$0.138**
- **Total cost for all lessons: 39 lessons × $0.138 = $5.38**

**Warm Cache (Ongoing Cost)**:
- With indefinite caching, only first generation hits OpenAI
- Subsequent requests: $0 (served from Postgres cache)
- Ongoing cost (100 users, 5 translations each): 500 requests × 0% (cache hit) = **$0**

**Total Hackathon Cost Estimate**: **~$5-10** (including testing and iterations)

**Note**: Urdu translation is the most cost-effective feature due to indefinite caching and single-variant translations.

---

## Estimated Timeline

- **Phase 0 (Research)**: 1.5 hours
- **Phase 1 (Design & Contracts)**: 2 hours
- **Phase 2 (Task Breakdown)**: 30 minutes (/sp.tasks)
- **Phase 3-7 (Implementation)**: 8-12 hours (/sp.implement)
  - Database migrations: 1 hour
  - Technical terms list: 1 hour
  - Translation service: 2-3 hours
  - LLM prompt engineering: 2 hours
  - API endpoints: 1 hour
  - Frontend components: 2-3 hours
  - RTL CSS styling: 1-2 hours
  - Testing: 1-2 hours
- **Total**: ~12-16 hours (1.5-2 days of focused work)

---

## Integration with Other Features

### Better Auth Integration (Optional)
- **User Language Preference**: Add `preferred_language` field to user_profiles (values: "en", "ur")
- **Auto-Translation**: Automatically show Urdu if user's preference is "ur"
- **Translation History**: Track which lessons user has translated (analytics)

### Content Personalization Integration (Future)
- **Personalized + Translated**: First personalize (adapt to user background), then translate to Urdu
- **Cache Layering**: Cache both personalized English and personalized Urdu versions

### RAG Chatbot Enhancement (Future)
- **Urdu Chat**: Allow users to ask questions in Urdu, retrieve from Urdu-translated content
- **Embeddings**: Optionally embed Urdu translations into Qdrant (separate collection)

---

## Urdu Font Recommendations

### Primary Font
- **Noto Nastaliq Urdu** (Google Fonts) - Modern, readable, open-source

### Fallback Fonts
- **Jameel Noori Nastaleeq** - Traditional Nastaliq style
- **Alvi Nastaleeq** - Classic calligraphic style
- **Nafees Nastaleeq** - Open-source alternative

### Font Stack
```css
font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", "Alvi Nastaleeq", "Nafees Nastaleeq", serif;
```

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown in `specs/urdu-translation/tasks.md`
2. Execute Phase 0 research tasks (RTL best practices, technical term preservation, Urdu typography)
3. Execute Phase 1 design tasks (data-model.md, contracts/translation-api-spec.yaml, quickstart.md)
4. Begin implementation via `/sp.implement` command

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` command
**Constitution Compliance**: ✅ PASS
**Output**: `plan.md` (this file), `research.md` (to be generated in Phase 0)
**Estimated Cost**: ~$5-10 (OpenAI API) - Most cost-effective feature
**Estimated Timeline**: 12-16 hours (1.5-2 days)
