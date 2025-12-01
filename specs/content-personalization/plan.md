# Implementation Plan: Content Personalization

**Created Using**: SpecKit Plus + Claude Code  
**Branch**: `main` | **Date**: 2025-11-29 | **Feature**: Content Personalization (Bonus - 50 points)  
**Implemented By**: Claude (Anthropic) + SpecKit Plus  
**Completion Date**: 2025-12-01  
**Input**: Hackathon requirements for adaptive content based on user background

## Summary

Build an intelligent content personalization system that adapts Physical AI textbook chapters to individual learner backgrounds. The system will:
- Personalize chapter content based on user profile (software/hardware experience, learning goals)
- Provide a "Personalize for Me" button at the start of each chapter
- Use OpenAI LLM to adapt content difficulty, examples, and explanations
- Cache personalized content to reduce API costs and improve performance
- Maintain original content structure while adapting language and examples

**Technical Approach**: FastAPI backend with OpenAI GPT-4 for content adaptation, Neon Postgres for caching personalized versions, React component in Docusaurus for personalization UI.

## Technical Context

**Language/Version**: Python 3.11+ (FastAPI backend), TypeScript/React 18 (Docusaurus frontend)
**Primary Dependencies**: FastAPI, OpenAI Python SDK, SQLAlchemy, Pydantic, React, markdown-it (Markdown parsing)
**Storage**: Neon Postgres (personalized_content cache, personalization_requests tracking)
**Testing**: pytest (backend unit/integration tests), Jest (frontend component tests)
**Target Platform**: Cloud-deployed API (Vercel/Railway/Render) + Static Docusaurus site (GitHub Pages/Netlify)
**Project Type**: Web (backend API + frontend component)
**Performance Goals**:
- First personalization (cold): < 10 seconds (LLM generation)
- Cached personalization (warm): < 500ms (database retrieval)
- Cache hit rate: > 80% (for common user profiles)
**Constraints**:
- Must preserve Markdown/MDX structure (headers, code blocks, components)
- Must maintain CEFR proficiency levels (not oversimplify)
- Must not break interactive components (InteractivePython, Quiz, TryWithAI)
- OpenAI API costs must be minimized via caching
**Scale/Scope**:
- 39 lessons to personalize
- ~5-10 personalization variants per lesson (based on user profile combinations)
- 100s of users (hackathon scale)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Phase-Driven Architecture**: This is Phase 2 (RAG + Features) - Content personalization is bonus feature
✅ **Spec-Driven Development**: Following `/sp.plan` → `/sp.implement` workflow (backend development path)
✅ **Backend Development Principles**: Using FastAPI, Neon Postgres, proper error handling, security standards
✅ **Code Quality Standards**: Python 3.11+ with type hints, comprehensive docstrings, test coverage > 70%
✅ **API Design Standards**: RESTful conventions, consistent response formats, proper HTTP status codes
✅ **Security Requirements**: Environment variables for secrets, rate limiting (protect OpenAI API), input validation
✅ **File Organization**: Backend follows `backend/app/{main,config,models,api,services,utils}` structure
✅ **Educational Excellence**: Personalization must maintain pedagogical quality, not sacrifice accuracy for simplicity

**No violations detected. All gates PASS.**

## Project Structure

### Documentation (this feature)

```text
specs/content-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0: LLM prompting strategies, caching patterns
├── data-model.md        # Phase 1: Database schemas (personalized_content, requests)
├── quickstart.md        # Phase 1: Setup instructions for personalization service
├── contracts/           # Phase 1: API contracts (personalization endpoints)
│   └── personalization-api-spec.yaml
└── tasks.md             # Phase 2: Created by /sp.tasks command
```

### Source Code (repository root)

```text
backend/                          # FastAPI backend
├── app/
│   ├── main.py                  # FastAPI app (add personalization routes)
│   ├── config.py                # Add OpenAI config for personalization
│   ├── models/
│   │   ├── personalized_content.py  # SQLAlchemy model for cache
│   │   └── personalization_request.py # SQLAlchemy model for tracking
│   ├── api/
│   │   └── personalization_routes.py # /api/personalize endpoints
│   ├── services/
│   │   ├── personalization_service.py # Core personalization logic
│   │   ├── content_adapter.py   # LLM-based content adaptation
│   │   ├── cache_service.py     # Caching strategy (profile hashing)
│   │   └── markdown_processor.py # Preserve MDX structure during adaptation
│   └── utils/
│       ├── profile_hasher.py    # Generate cache keys from user profiles
│       └── prompt_templates.py  # LLM prompts for personalization
├── tests/
│   ├── unit/
│   │   ├── test_personalization_service.py
│   │   ├── test_content_adapter.py
│   │   └── test_cache_service.py
│   └── integration/
│       └── test_personalization_api.py
└── alembic/
    └── versions/
        └── 0003_create_personalization_tables.py

book-source/                      # Existing Docusaurus site
└── src/
    └── components/
        ├── Personalization/
        │   ├── PersonalizeButton.tsx  # "Personalize for Me" button
        │   ├── PersonalizedContent.tsx # Display personalized version
        │   ├── PersonalizationToggle.tsx # Switch between original/personalized
        │   ├── LoadingIndicator.tsx  # Loading state during LLM generation
        │   └── types.ts              # TypeScript interfaces
        └── ChapterHeader.tsx         # Modified to include PersonalizeButton
```

**Structure Decision**: Extend existing backend with personalization service. Frontend components integrate into chapter pages without breaking existing layout.

## Complexity Tracking

> No violations detected - this section is empty.

---

## Phase 0: Research & Technology Decisions

**Prerequisites**: Constitution approved
**Output**: `research.md` with all technology choices documented

### Research Tasks

1. **LLM Prompting Strategies for Content Adaptation**
   - Research: Few-shot prompting, chain-of-thought for educational content adaptation
   - Decision needed: Prompt structure, example selection, output format constraints
   - Reference: OpenAI best practices for educational content, prompt engineering guides

2. **Content Personalization Patterns**
   - Research: Adaptive learning systems, difficulty scaling strategies
   - Decision needed: What to adapt (language complexity, examples, analogies, depth of explanations)
   - Reference: Educational technology research, adaptive learning platforms

3. **Markdown/MDX Preservation Strategies**
   - Research: How to preserve structure while adapting text (headers, code blocks, components)
   - Decision needed: Parsing strategy (AST-based vs regex), component handling
   - Reference: Markdown parsers, MDX specification

4. **Caching Strategy for Personalized Content**
   - Research: Cache key generation from user profiles, cache invalidation strategies
   - Decision needed: Profile hashing algorithm, cache TTL (time-to-live), storage format
   - Reference: Caching best practices, database optimization

5. **User Profile to Personalization Mapping**
   - Research: How to map user background (software/hardware experience) to content adaptations
   - Decision needed: Personalization dimensions (technical depth, analogies, prerequisites)
   - Reference: Learning science, zone of proximal development

6. **OpenAI Cost Optimization**
   - Research: Token usage minimization, batch processing, model selection (GPT-4 vs GPT-3.5)
   - Decision needed: Which model to use, token limits per personalization, fallback strategies
   - Reference: OpenAI pricing, cost optimization techniques

7. **Personalization Quality Validation**
   - Research: How to validate LLM output maintains educational quality
   - Decision needed: Validation rules (CEFR level preservation, accuracy checks, completeness)
   - Reference: Educational content validation, LLM output validation

### Research Outputs

All findings documented in `specs/content-personalization/research.md` with format:
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
**Output**: `data-model.md`, `contracts/personalization-api-spec.yaml`, `quickstart.md`

### 1.1 Data Model Design (`data-model.md`)

**Postgres Schema (Neon)**:

```sql
-- Personalized content cache (stores adapted chapter versions)
CREATE TABLE personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(50) NOT NULL,  -- e.g., "chapter-01"
    lesson_id VARCHAR(50) NOT NULL,   -- e.g., "lesson-01-introduction"
    profile_hash VARCHAR(64) NOT NULL,  -- SHA-256 hash of user profile
    original_content TEXT NOT NULL,   -- Original Markdown content
    personalized_content TEXT NOT NULL,  -- Adapted Markdown content
    personalization_params JSONB NOT NULL,  -- User profile used for adaptation
    model_used VARCHAR(50),  -- e.g., "gpt-4-turbo-preview"
    tokens_used INTEGER,  -- OpenAI token count
    generation_time_ms INTEGER,  -- Time to generate (for analytics)
    created_at TIMESTAMP DEFAULT NOW(),
    expires_at TIMESTAMP,  -- Cache TTL (30 days default)
    UNIQUE(chapter_id, lesson_id, profile_hash)
);

-- Personalization requests (tracking and analytics)
CREATE TABLE personalization_requests (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),  -- Optional: link to authenticated user
    chapter_id VARCHAR(50) NOT NULL,
    lesson_id VARCHAR(50) NOT NULL,
    profile_hash VARCHAR(64) NOT NULL,
    cache_hit BOOLEAN DEFAULT FALSE,  -- Was content served from cache?
    response_time_ms INTEGER,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Indexes
CREATE INDEX idx_personalized_content_lookup ON personalized_content(chapter_id, lesson_id, profile_hash);
CREATE INDEX idx_personalized_content_expires ON personalized_content(expires_at);
CREATE INDEX idx_personalization_requests_user ON personalization_requests(user_id);
CREATE INDEX idx_personalization_requests_chapter ON personalization_requests(chapter_id);
```

**Profile Hashing Algorithm**:

```python
import hashlib
import json

def generate_profile_hash(user_profile: dict) -> str:
    """
    Generate deterministic cache key from user profile.

    Profile structure:
    {
        "software_experience": "intermediate",
        "hardware_experience": "hobbyist",
        "learning_goals": "build humanoid robots",
        "preferred_topics": ["ros2", "gazebo"]
    }

    Returns: SHA-256 hash (64 characters)
    """
    # Sort keys for deterministic hashing
    canonical = json.dumps(user_profile, sort_keys=True)
    return hashlib.sha256(canonical.encode()).hexdigest()
```

### 1.2 API Contracts (`contracts/personalization-api-spec.yaml`)

**Endpoints**:

1. **POST /api/personalize/chapter** - Personalize a chapter
   ```yaml
   /api/personalize/chapter:
     post:
       summary: Generate personalized version of a chapter
       requestBody:
         content:
           application/json:
             schema:
               type: object
               required: [chapter_id, lesson_id, user_profile]
               properties:
                 chapter_id:
                   type: string
                   example: "chapter-01"
                 lesson_id:
                   type: string
                   example: "lesson-01-introduction"
                 user_profile:
                   type: object
                   properties:
                     software_experience:
                       type: string
                       enum: [beginner, intermediate, advanced]
                     hardware_experience:
                       type: string
                       enum: [none, hobbyist, professional]
                     learning_goals:
                       type: string
                     preferred_topics:
                       type: array
                       items:
                         type: string
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
                       personalized_content:
                         type: string
                         description: Adapted Markdown content
                       cache_hit:
                         type: boolean
                       metadata:
                         type: object
                         properties:
                           model_used: string
                           tokens_used: integer
                           generation_time_ms: integer
   ```

2. **GET /api/personalize/status/{chapter_id}/{lesson_id}** - Check if personalized version exists
3. **POST /api/personalize/preview** - Preview personalization without caching (for testing)
4. **DELETE /api/personalize/cache/clear** - Clear cache for a user (admin endpoint)

Full OpenAPI spec generated in `contracts/personalization-api-spec.yaml`.

### 1.3 Quickstart Guide (`quickstart.md`)

Step-by-step developer setup:
1. Configure OpenAI API key in environment variables
2. Run database migrations (personalized_content, personalization_requests tables)
3. Test LLM prompts with sample user profiles
4. Implement caching logic with profile hashing
5. Add PersonalizeButton component to chapter pages
6. Test personalization flow (button click → API call → display adapted content)

### 1.4 Agent Context Update

Run constitution-mandated agent context update:
```powershell
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

Adds Content Personalization technologies to agent context:
- OpenAI GPT-4 for content adaptation
- Markdown parsing libraries
- Caching strategies

---

## Phase 2: Task Breakdown

**Note**: This phase is executed by `/sp.tasks` command, NOT by `/sp.plan`.

The `/sp.tasks` command will generate `specs/content-personalization/tasks.md` with detailed implementation tasks based on this plan.

**Expected Task Structure**:
1. Database Schema & Migrations (personalized_content, personalization_requests tables)
2. Profile Hashing Utility (deterministic cache key generation)
3. Markdown/MDX Processor (preserve structure during adaptation)
4. LLM Prompt Templates (few-shot prompts for different user profiles)
5. Content Adapter Service (OpenAI integration, output validation)
6. Caching Service (retrieve/store personalized content, TTL management)
7. Personalization API Endpoints (POST /api/personalize/chapter, etc.)
8. Frontend PersonalizeButton Component (trigger personalization)
9. Frontend PersonalizedContent Component (display adapted version, toggle original/personalized)
10. Testing (Unit tests for content adapter, integration tests for API)
11. Cost Monitoring (track OpenAI token usage, optimize prompts)

---

## Phase 3-7: Implementation Phases

**Note**: These phases are executed by `/sp.implement` command, NOT by `/sp.plan`.

The `/sp.implement` command will execute tasks from `tasks.md` in dependency order, implementing:
- Phase 3: Database & Caching Infrastructure
- Phase 4: LLM Integration & Prompt Engineering
- Phase 5: Content Adaptation Service
- Phase 6: API Endpoints
- Phase 7: Frontend Components
- Phase 8: Testing & Validation
- Phase 9: Cost Optimization & Monitoring

---

## Architectural Decisions

### ADR-001: GPT-4 vs GPT-3.5 for Content Adaptation
**Decision**: Use GPT-4-turbo-preview for personalization
**Rationale**: Superior understanding of educational content, better instruction following, maintains technical accuracy. Cost offset by caching (80%+ hit rate expected).
**Alternatives**:
- GPT-3.5-turbo (rejected: lower quality adaptations, may lose technical nuance)
- Claude 3 Opus (rejected: OpenAI already used for RAG chatbot, consistency)

### ADR-002: Caching Strategy - Profile Hashing
**Decision**: Generate SHA-256 hash of user profile as cache key
**Rationale**: Deterministic, collision-resistant, enables exact match lookups. User profiles with identical values share cached content (cost savings).
**Alternatives**:
- Per-user caching (rejected: low cache hit rate, high OpenAI costs)
- Fuzzy matching (rejected: complexity, potential incorrect matches)

### ADR-003: Content Adaptation Scope
**Decision**: Adapt explanations, examples, analogies; preserve structure, code, components
**Rationale**: Maintains consistency with original lesson structure, avoids breaking interactive components, focuses adaptation where it provides most value.
**What to adapt**:
- Language complexity (sentence structure, vocabulary)
- Examples (software-focused vs hardware-focused)
- Analogies (based on user background)
- Depth of explanations (beginner vs advanced)
**What NOT to adapt**:
- Code blocks (preserve exact syntax)
- Interactive components (InteractivePython, Quiz, TryWithAI)
- Headers, structure, navigation
- Mathematical formulas

### ADR-004: Cache TTL (Time-to-Live)
**Decision**: 30-day cache expiration with manual invalidation option
**Rationale**: Balances freshness (lessons may be updated) with cost savings. 30 days sufficient for hackathon demo.
**Alternatives**:
- Indefinite caching (rejected: stale content if lessons updated)
- 7-day expiration (rejected: too short, unnecessary re-generations)

### ADR-005: Personalization Trigger
**Decision**: Opt-in "Personalize for Me" button at chapter start
**Rationale**: User consent, preserves original content as default, clear UX. Users who don't personalize see canonical content.
**Alternatives**:
- Automatic personalization (rejected: no user control, potential confusion)
- Per-section personalization (rejected: UI clutter, excessive API calls)

### ADR-006: MDX Preservation Strategy
**Decision**: Use AST-based Markdown parsing (remark/mdast) to preserve structure
**Rationale**: Reliable extraction of text nodes while preserving code blocks, components, frontmatter. Industry-standard approach.
**Alternatives**:
- Regex-based parsing (rejected: fragile, likely to break on edge cases)
- Full re-generation (rejected: loses structure, breaks components)

---

## LLM Prompt Design

### Base Prompt Template

```python
PERSONALIZATION_PROMPT = """
You are an expert educational content adapter specializing in Physical AI and robotics.

**Original Lesson Content:**
{original_content}

**User Profile:**
- Software Experience: {software_experience}
- Hardware Experience: {hardware_experience}
- Learning Goals: {learning_goals}

**Task:**
Adapt the lesson content to this user's background while following these rules:

1. **Preserve Structure:**
   - Keep all headers (# ## ###) exactly as they are
   - Keep all code blocks unchanged
   - Keep all interactive components (:::note, :::tip, <InteractivePython>, etc.)
   - Keep all mathematical formulas unchanged

2. **Adapt Content:**
   - Adjust language complexity to match software experience level
   - Use examples relevant to their hardware experience
   - Provide analogies based on their background
   - Adjust depth of explanations (more detail for beginners, concise for advanced)

3. **Maintain Quality:**
   - Preserve technical accuracy
   - Maintain CEFR proficiency level (do not oversimplify)
   - Keep the same pedagogical structure (Foundation → Application → Integration → Innovation)

**Output Format:**
Return ONLY the adapted Markdown content. Do not add explanations or comments.

**Example Adaptations:**

For software_experience = "beginner":
- Use simple analogies (e.g., "Think of ROS 2 nodes like functions in a program")
- Explain technical terms (e.g., "A topic is like a message channel where nodes communicate")

For hardware_experience = "professional":
- Use hardware analogies (e.g., "ROS 2 topics are similar to I2C bus communication")
- Reference real robot hardware (e.g., "similar to the CAN bus in industrial robots")

Now adapt the content:
"""
```

---

## Dependencies

### External Services
- **Neon Postgres**: Free tier (3GB storage, shared compute) - already provisioned
- **OpenAI API**: Pay-as-you-go (GPT-4 completions)

### Python Packages (backend/requirements.txt additions)
```
markdown-it-py==3.0.0  # Markdown parsing
mdurl==0.1.2  # Markdown URL handling
remark-parse==10.0.1  # MDX parsing (if using Node.js-based approach)
```

### Frontend Packages (book-source/package.json additions)
```json
{
  "dependencies": {
    "react-markdown": "^9.0.1",  // Render personalized Markdown
    "remark-gfm": "^4.0.0"  // GitHub Flavored Markdown support
  }
}
```

---

## Success Criteria

✅ **Functional**:
- [ ] Users can click "Personalize for Me" button on any chapter
- [ ] Personalized content adapts to user profile (software/hardware experience)
- [ ] Original content structure preserved (headers, code, components)
- [ ] Users can toggle between original and personalized versions
- [ ] Personalized content cached (subsequent requests < 500ms)

✅ **Performance**:
- [ ] First personalization (cold): < 10 seconds
- [ ] Cached personalization (warm): < 500ms
- [ ] Cache hit rate: > 80%
- [ ] OpenAI token usage: < 5000 tokens per personalization

✅ **Quality**:
- [ ] Personalized content maintains technical accuracy
- [ ] CEFR proficiency levels preserved
- [ ] No broken Markdown/MDX (headers, code blocks, components)
- [ ] Test coverage > 70% (content adapter, caching)

✅ **Cost Optimization**:
- [ ] Caching reduces OpenAI API costs by > 80%
- [ ] Token usage monitored and logged
- [ ] Fallback to original content if LLM fails

✅ **UX**:
- [ ] Clear loading indicator during personalization
- [ ] Error messages if personalization fails
- [ ] Smooth toggle between original/personalized
- [ ] Personalization button visually prominent but not intrusive

---

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| LLM hallucination (inaccurate content) | Medium | High | Validation rules, output checks, manual review for first few chapters |
| High OpenAI API costs | Medium | Medium | Aggressive caching (80%+ hit rate), token usage monitoring, budget limits |
| MDX parsing failures | Medium | Medium | Robust AST-based parsing, fallback to original content |
| Personalization quality inconsistency | Medium | Medium | Few-shot prompting, prompt engineering iteration, user feedback |
| Cache storage growth | Low | Low | 30-day TTL, monitor Postgres usage, compression |

---

## Cost Estimation

### OpenAI API Costs (GPT-4-turbo-preview)

**Assumptions**:
- 39 lessons to personalize
- ~5 unique user profile combinations (beginner/intermediate/advanced × none/hobbyist/professional)
- Average lesson length: 3000 tokens
- GPT-4-turbo pricing: $0.01/1K input tokens, $0.03/1K output tokens

**Cold Cache (First Generation)**:
- Input tokens per lesson: 3000 tokens (lesson content) + 200 tokens (prompt) = 3200 tokens
- Output tokens per lesson: ~3500 tokens (adapted content)
- Cost per lesson: (3.2K × $0.01) + (3.5K × $0.03) = $0.032 + $0.105 = **$0.137**
- Cost for all lessons × profiles: 39 lessons × 5 profiles × $0.137 = **$26.72**

**Warm Cache (80% hit rate)**:
- With caching, only 20% of requests hit OpenAI
- Ongoing cost (100 users, 5 personalizations each): 500 requests × 20% × $0.137 = **$13.70**

**Total Hackathon Cost Estimate**: ~$40-50 (including testing and iterations)

---

## Estimated Timeline

- **Phase 0 (Research)**: 2 hours
- **Phase 1 (Design & Contracts)**: 2 hours
- **Phase 2 (Task Breakdown)**: 30 minutes (/sp.tasks)
- **Phase 3-7 (Implementation)**: 10-14 hours (/sp.implement)
  - Database migrations: 1 hour
  - Profile hashing & caching: 2 hours
  - LLM prompt engineering: 3-4 hours (iterative testing)
  - Content adapter service: 2-3 hours
  - API endpoints: 1-2 hours
  - Frontend components: 2-3 hours
  - Testing: 2 hours
- **Total**: ~15-19 hours (2-2.5 days of focused work)

---

## Integration with Other Features

### Better Auth Integration (Required)
- **User Profile Access**: Retrieve software_experience, hardware_experience, learning_goals from user_profiles table
- **Authentication**: Personalization endpoints can be protected (require login) or public (use local profile)
- **Profile Hash Caching**: Authenticated users benefit from shared cache (same profile → same hash)

### RAG Chatbot Enhancement
- **Context-Aware Responses**: Use personalized content as retrieval source (if user has personalized)
- **Embeddings**: Optionally embed personalized versions into Qdrant (future enhancement)

### Progress Tracking (Future)
- **Personalization History**: Track which lessons user has personalized (store in user_profiles.completed_lessons)

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown in `specs/content-personalization/tasks.md`
2. Execute Phase 0 research tasks (LLM prompting, caching strategies, MDX preservation)
3. Execute Phase 1 design tasks (data-model.md, contracts/personalization-api-spec.yaml, quickstart.md)
4. Begin implementation via `/sp.implement` command

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` command
**Constitution Compliance**: ✅ PASS
**Output**: `plan.md` (this file), `research.md` (to be generated in Phase 0)
**Estimated Cost**: ~$40-50 (OpenAI API)
**Estimated Timeline**: 15-19 hours (2-2.5 days)
