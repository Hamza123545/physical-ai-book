---
id: 0001
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: main
branch: main
user: system
command: /sp.plan
labels: ["rag", "chatbot", "fastapi", "qdrant", "neon-postgres", "openai", "phase-2"]
links:
  spec: null
  ticket: null
  adr: specs/main/plan.md#architectural-decisions
  pr: null
files:
  - specs/main/plan.md
tests:
  - None (planning phase)
---

## Prompt

Create implementation plan for RAG Chatbot (Base - 100 points).

**Context**: Physical AI textbook backend (FastAPI, Neon Postgres, Qdrant Cloud, OpenAI)

**Requirements**:
- Embed all book content (book-source/docs/*.md) into Qdrant
- Support general queries + selected text queries
- API: POST /api/chat, POST /api/chat/selected-text, GET /api/chat/history, POST /api/chat/clear, POST /api/embeddings/ingest
- React chatbot component in Docusaurus frontend
- Store chat history in Postgres

**Constitution**: Follow .specify/memory/constitution.md (Backend Development Principles)

**Output**: Detailed plan with 7 phases (Infrastructure, Ingestion, RAG Core, Selected Text, API, Frontend, Testing). Include dependencies, estimated time, file structure.

## Response snapshot

Created comprehensive implementation plan (`specs/main/plan.md`) for RAG Chatbot following constitution-mandated backend development workflow.

**Key Outputs**:

1. **Technical Context** - Defined stack (Python 3.11+, FastAPI, Qdrant Cloud, Neon Postgres, OpenAI, React/TypeScript)
2. **Constitution Check** - All gates PASS (Phase-Driven Architecture, Backend Development Principles, Security Requirements)
3. **Project Structure** - Defined backend/ and book-source/src/components/ChatBot/ organization
4. **Phase 0: Research** - 7 research tasks (Qdrant integration, OpenAI embeddings, MDX processing, schema design, selected text queries, CORS, rate limiting)
5. **Phase 1: Design** - Data models (Postgres tables, Qdrant collection schema), API contracts (5 endpoints), quickstart guide
6. **Architectural Decisions** - 5 ADRs documented (Qdrant Cloud, OpenAI text-embedding-3-small, 512-token chunks with 100-token overlap, full chat history retention, on-the-fly selected text embedding)
7. **Dependencies** - External services (Neon, Qdrant, OpenAI), Python packages (FastAPI, SQLAlchemy, Qdrant client), frontend packages
8. **Success Criteria** - Functional (embeddings, queries, chat history), Performance (< 3s responses, < 500ms search), Quality (70% test coverage), Security (CORS, rate limiting, input validation)
9. **Risk Analysis** - OpenAI quota, Qdrant limits, MDX parsing, CORS, DB growth
10. **Timeline** - 24-30 hours (3-4 days) total estimated

**Database Schemas Defined**:
- Postgres: `chat_sessions`, `chat_messages` with indexes
- Qdrant: `physical_ai_textbook` collection (1536-dim vectors, cosine distance, payload: chapter, lesson, section_title, content, file_path, chunk_index, cefr_level)

**API Endpoints Designed**:
- POST /api/chat (general questions)
- POST /api/chat/selected-text (context-specific questions)
- GET /api/chat/history/{session_id}
- POST /api/chat/clear
- POST /api/embeddings/ingest

**Next Steps**:
1. Run `/sp.tasks` to generate tasks.md
2. Execute Phase 0 research
3. Execute Phase 1 design (data-model.md, contracts/api-spec.yaml, quickstart.md)
4. Begin implementation via `/sp.implement`

## Outcome

- ✅ Impact: Comprehensive plan created for 100-point base hackathon requirement (RAG Chatbot)
- 🧪 Tests: None (planning phase - tests defined in plan)
- 📁 Files: specs/main/plan.md (535 lines)
- 🔁 Next prompts: `/sp.tasks` → Phase 0 research → Phase 1 design → `/sp.implement`
- 🧠 Reflection: Plan follows constitution Backend Development Principles. All architecture decisions documented with rationale. Clear separation of concerns (backend/, book-source/). Realistic timeline (24-30 hours). No NEEDS CLARIFICATION items - all technical context resolved through research tasks.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Execute `/sp.tasks` to validate plan completeness by generating actionable task breakdown
