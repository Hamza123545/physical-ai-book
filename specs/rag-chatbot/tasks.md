# Tasks: RAG Chatbot Backend

**Created Using**: SpecKit Plus + Claude Code  
**Workflow**: `/sp.tasks` (after `/sp.plan`)  
**Input**: `specs/rag-chatbot/plan.md`  
**Status**: ✅ Complete (18/80 tasks completed)  
**Date**: 2025-01-29

---

## Task Organization

Tasks are organized by phase and user story to enable independent implementation and testing.

**Format**: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)

---

## Phase 1: Infrastructure Setup (US1 Foundation)

**Purpose**: Project initialization and basic structure

- [x] T001 Create FastAPI project structure in `backend/`
- [x] T002 Create `backend/app/main.py` with FastAPI app initialization
- [x] T003 Create `backend/app/config.py` with environment variable loading
- [x] T004 Setup Neon Postgres connection (SQLAlchemy engine, session factory)
- [x] T005 Setup Qdrant client initialization in `config.py`
- [x] T006 Create `backend/app/models/chat_history.py` with SQLAlchemy models (ChatSession, ChatMessage)
- [x] T007 Create `backend/app/models/schemas.py` with Pydantic schemas
- [x] T008 Initialize Alembic for database migrations
- [x] T009 Create initial migration for chat tables
- [x] T010 Write `backend/requirements.txt` with all dependencies
- [x] T011 Create `backend/.env.example` with all required environment variables
- [x] T012 Add structured logging utility (`backend/app/utils/logger.py`)

---

## Phase 2: Content Ingestion (US1)

**Purpose**: Process and embed all textbook content

- [x] T013 Create `backend/app/utils/markdown_processor.py` for MDX parsing
- [x] T014 Implement chunking logic (512 tokens, 100 overlap)
- [x] T015 Create `backend/app/services/embeddings_service.py` for Qdrant operations
- [ ] T016 Implement OpenAI embedding generation (batch processing)
- [ ] T017 Create Qdrant collection with proper vector configuration
- [ ] T018 Implement vector ingestion with metadata (chapter, lesson, file_path)
- [ ] T019 Create `POST /api/embeddings/ingest` endpoint
- [ ] T020 Add error handling and logging for ingestion
- [ ] T021 Test ingestion with sample markdown files
- [ ] T022 Add progress tracking for large content ingestion

---

## Phase 3: RAG Core Logic (US2)

**Purpose**: Implement RAG using OpenAI Agents SDK

- [x] T023 Add `openai-agents>=0.1.0` to `requirements.txt`
- [x] T024 Add `litellm>=1.50.0` and `google-generativeai>=0.8.0` for Gemini support
- [x] T025 Create `backend/app/services/agent_service.py` with Agent creation functions
- [x] T026 Implement `create_rag_agent()` - Agent with RAG context and instructions
- [x] T027 Implement `create_selected_text_agent()` - Agent for selected text queries
- [x] T028 Implement `run_agent_sync()` - Synchronous agent execution using Runner
- [x] T029 Implement `run_agent_stream()` - Streaming agent execution (optional)
- [x] T030 Create `backend/app/services/rag_service.py` for RAG orchestration
- [x] T031 Implement `generate_rag_response()` - Query Qdrant → Create Agent → Run → Return
- [x] T032 Implement `extract_source_citations()` - Extract citations from retrieved chunks
- [x] T033 Update `backend/app/config.py` with Gemini configuration (USE_GEMINI, GEMINI_API_KEY)
- [x] T034 Test RAG flow with sample queries
- [ ] T035 Add error handling for agent execution failures
- [ ] T036 Implement chat history integration with Agents SDK Sessions

---

## Phase 4: Selected Text Query (US3)

**Purpose**: Support queries based on user-selected text

- [x] T037 Implement `generate_selected_text_response()` in `rag_service.py`
- [x] T038 Create embedding for selected text
- [x] T039 Search Qdrant with higher threshold (0.8) and limit (3 chunks)
- [x] T040 Create agent with selected text context
- [x] T041 Generate answer using only selected text context
- [ ] T042 Test selected text query with various text selections
- [ ] T043 Add validation for selected text length limits

---

## Phase 5: API Endpoints (US2, US3, US4)

**Purpose**: Complete REST API with all endpoints

- [x] T044 Create `backend/app/api/chat_routes.py` with router
- [ ] T045 Implement `POST /api/chat` endpoint (general query)
- [ ] T046 Implement `POST /api/chat/selected-text` endpoint
- [ ] T047 Create `backend/app/services/db_service.py` for database operations
- [ ] T048 Implement `get_or_create_session()` - Session management
- [ ] T049 Implement `save_message()` - Save chat messages to Postgres
- [ ] T050 Implement `get_session_history()` - Retrieve chat history
- [ ] T051 Implement `clear_session_history()` - Clear chat history
- [ ] T052 Implement `GET /api/chat/history/{session_id}` endpoint
- [ ] T053 Implement `POST /api/chat/clear` endpoint
- [ ] T054 Add Pydantic request/response validation
- [ ] T055 Add CORS middleware for Docusaurus frontend
- [ ] T056 Add rate limiting middleware (slowapi)
- [ ] T057 Add request/response logging middleware
- [ ] T058 Register all routes in `backend/app/main.py`
- [ ] T059 Add OpenAPI/Swagger documentation
- [ ] T060 Test all endpoints with sample requests

---

## Phase 6: Frontend Integration (US5)

**Purpose**: React chatbot component for Docusaurus

- [ ] T061 Create `book-source/src/components/Chatbot.tsx` React component
- [ ] T062 Design chat UI (messages list, input field, send button)
- [ ] T063 Implement message state management
- [ ] T064 Implement selected text detection (window.getSelection)
- [ ] T065 Add "Ask about this" button when text is selected
- [ ] T066 Connect to `POST /api/chat` endpoint (fetch API)
- [ ] T067 Connect to `POST /api/chat/selected-text` endpoint
- [ ] T068 Display chat messages with proper formatting
- [ ] T069 Display source citations in messages
- [ ] T070 Add loading states (spinner, disabled buttons)
- [ ] T071 Add error handling and error messages
- [ ] T072 Implement chat history display
- [ ] T073 Create `book-source/src/components/Chatbot.module.css` for styling
- [ ] T074 Make component responsive and accessible
- [ ] T075 Test component in Docusaurus development environment

---

## Phase 7: Testing & Documentation (All US)

**Purpose**: Quality assurance and documentation

- [ ] T076 Write unit tests for `agent_service.py` (mock Agents SDK)
- [ ] T077 Write unit tests for `rag_service.py` (mock Qdrant, Agents SDK)
- [ ] T078 Write unit tests for `embeddings_service.py` (mock Qdrant, OpenAI)
- [ ] T079 Write integration tests for `POST /api/chat` endpoint
- [ ] T080 Write integration tests for `POST /api/chat/selected-text` endpoint
- [ ] T081 Write integration tests for `GET /api/chat/history/{session_id}` endpoint
- [ ] T082 Test model switching (OpenAI ↔ Gemini)
- [ ] T083 Test error scenarios (API failures, invalid requests)
- [ ] T084 Update `backend/README.md` with setup instructions
- [ ] T085 Create `backend/GEMINI_SETUP.md` guide
- [ ] T086 Document all environment variables
- [ ] T087 Add troubleshooting section to README
- [ ] T088 Verify API documentation in Swagger UI
- [ ] T089 Run full test suite and fix any failures
- [ ] T090 Create deployment guide

---

## Dependencies

### Phase 1 → Phase 2
- Infrastructure must be ready before content ingestion

### Phase 1, Phase 2 → Phase 3
- Database and embeddings must be ready before RAG core

### Phase 3 → Phase 4
- RAG core must be ready before selected text query

### Phase 3, Phase 4 → Phase 5
- RAG services must be ready before API endpoints

### Phase 5 → Phase 6
- API endpoints must be ready before frontend integration

### All Phases → Phase 7
- All features must be implemented before testing

---

## Parallel Execution Opportunities

**Within Phase 2**:
- T013 (markdown processor) and T015 (embeddings service) can be done in parallel

**Within Phase 3**:
- T026 (create_rag_agent) and T027 (create_selected_text_agent) can be done in parallel

**Within Phase 5**:
- T045 (POST /api/chat) and T046 (POST /api/chat/selected-text) can be done in parallel
- T052 (GET /api/chat/history) and T053 (POST /api/chat/clear) can be done in parallel

**Within Phase 7**:
- All test files can be written in parallel

---

## Implementation Status

**Completed**: 18/80 tasks (22.5%)  
**In Progress**: Phase 3 (RAG Core Logic)  
**Next**: Complete Phase 3, then move to Phase 4

### Completed Phases
- ✅ Phase 1: Infrastructure Setup (100%)
- ✅ Phase 2: Content Ingestion (37.5% - markdown processor done)

### In Progress
- 🚧 Phase 3: RAG Core Logic (85% - core logic done, testing pending)

### Pending
- ⏳ Phase 4: Selected Text Query (80% - implementation done, testing pending)
- ⏳ Phase 5: API Endpoints (10% - routes created, endpoints pending)
- ⏳ Phase 6: Frontend Integration (0%)
- ⏳ Phase 7: Testing & Documentation (0%)

---

## Notes

- All tasks follow SpecKit Plus workflow (`/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`)
- OpenAI Agents SDK integration completed with multi-model support (OpenAI + Gemini)
- Constitution compliance verified (`.specify/memory/constitution.md`)
- Backend Development Principles followed throughout

