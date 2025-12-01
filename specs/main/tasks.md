# Tasks: RAG Chatbot for Physical AI Textbook

**Created Using**: SpecKit Plus + Claude Code  
**Implemented By**: Claude (Anthropic) + SpecKit Plus  
**Completion Date**: 2025-12-01  
**Input**: Design documents from `/specs/main/`
**Prerequisites**: plan.md (created), spec.md (requirements documented in plan)

**Tests**: Not explicitly requested - focused on implementation and integration

**Organization**: Tasks are grouped by functional capability (user story) to enable independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (FastAPI structure)
- **Frontend**: `book-source/src/components/ChatBot/`
- Paths follow web app structure as defined in plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure: backend/app/{main,config,models,api,services,utils}
- [X] T002 Initialize Python project with FastAPI, SQLAlchemy, Qdrant Client, OpenAI SDK in backend/requirements.txt
- [X] T003 [P] Create .env.example with DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY in backend/.env.example
- [X] T004 [P] Configure CORS middleware for GitHub Pages origin in backend/app/main.py
- [X] T005 [P] Setup structured logging utility in backend/app/utils/logger.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup Neon Postgres database and configure connection pooling in backend/app/config.py
- [X] T007 Create Alembic migrations framework in backend/alembic/
- [X] T008 [P] Setup Qdrant Cloud connection and initialize client in backend/app/config.py
- [X] T009 [P] Setup OpenAI client with API key management in backend/app/config.py
- [X] T010 Create database models: chat_sessions table with UUID, user_id, timestamps in backend/app/models/chat_history.py
- [X] T011 Create database models: chat_messages table with session_id FK, role, content, selected_text, retrieved_chunks JSONB in backend/app/models/chat_history.py
- [X] T012 [P] Create Pydantic schemas for request/response validation in backend/app/models/schemas.py
- [X] T013 Run Alembic migration to create chat_sessions and chat_messages tables
- [X] T014 [P] Create Qdrant collection 'physical_ai_textbook' with 1536-dim vectors, cosine distance in backend/app/services/embeddings_service.py
- [X] T015 [P] Implement rate limiting middleware (10 requests/min per session) using slowapi in backend/app/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Embeddings Ingestion (Priority: P1) 🎯

**Goal**: Embed all 39 book lessons into Qdrant for semantic search

**Independent Test**: POST /api/embeddings/ingest completes successfully, Qdrant collection contains ~300-500 vectors

### Implementation for User Story 1

- [X] T016 [P] [US1] Create Markdown processor to parse MDX files and extract text content in backend/app/utils/markdown_processor.py
- [X] T017 [P] [US1] Implement chunking strategy: 512 tokens per chunk, 100-token overlap in backend/app/utils/markdown_processor.py
- [X] T018 [P] [US1] Implement metadata extraction (chapter, lesson, section_title, cefr_level, file_path) in backend/app/utils/markdown_processor.py
- [X] T019 [US1] Create embeddings service: batch embed chunks using OpenAI text-embedding-3-small in backend/app/services/embeddings_service.py
- [X] T020 [US1] Implement Qdrant ingestion: store vectors with payload schema (chapter, lesson, section_title, content, file_path, chunk_index, cefr_level) in backend/app/services/embeddings_service.py
- [X] T021 [US1] Create POST /api/embeddings/ingest endpoint in backend/app/api/embeddings_routes.py
- [X] T022 [US1] Implement ingestion loop: read all book-source/docs/**/*.md files, chunk, embed, store in Qdrant
- [X] T023 [US1] Add progress logging and error handling for failed embeddings

**Checkpoint**: At this point, all book content should be embedded in Qdrant (verify with Qdrant dashboard: ~300-500 vectors)

---

## Phase 4: User Story 2 - General Chat Queries (Priority: P2) 🎯 MVP

**Goal**: Users can ask general questions about the textbook and receive RAG-based answers with source citations

**Independent Test**: POST /api/chat with {"session_id": "test-123", "message": "What is ROS 2?"} returns relevant answer with sources

### Implementation for User Story 2

- [X] T024 [P] [US2] Implement vector search: retrieve top-k relevant chunks from Qdrant (k=5, score > 0.7) in backend/app/services/embeddings_service.py
- [X] T025 [P] [US2] Create OpenAI service wrapper for chat completions (GPT-4-turbo) in backend/app/services/openai_service.py
- [X] T026 [US2] Implement RAG service: retrieve context + generate answer using OpenAI in backend/app/services/rag_service.py
- [X] T027 [US2] Design RAG prompt template: system message + retrieved chunks + user question in backend/app/services/rag_service.py
- [X] T028 [US2] Implement source citation extraction: map retrieved chunks to chapter/lesson/section in backend/app/services/rag_service.py
- [X] T029 [US2] Create database service for chat history: save user message and assistant response to Postgres in backend/app/services/db_service.py
- [X] T030 [US2] Create POST /api/chat endpoint with session_id and message validation in backend/app/api/chat_routes.py
- [X] T031 [US2] Implement chat flow: retrieve context → generate answer → save history → return response with sources
- [X] T032 [US2] Add error handling for OpenAI API failures (fallback: return "I'm having trouble answering that")
- [X] T033 [US2] Add error handling for Qdrant timeouts (fallback: "Search service temporarily unavailable")

**Checkpoint**: At this point, general chat queries should work end-to-end (test with sample questions)

---

## Phase 5: User Story 3 - Selected Text Queries (Priority: P3)

**Goal**: Users can select text from a lesson and ask questions specifically about that selection

**Independent Test**: POST /api/chat/selected-text with {"session_id": "test-123", "selected_text": "ROS 2 nodes communicate...", "question": "How?"} returns answer based only on selected text

### Implementation for User Story 3

- [ ] T034 [P] [US3] Implement selected text embedding: embed user's selected text on-the-fly in backend/app/services/rag_service.py
- [ ] T035 [US3] Create selected text query handler: inject selected text into prompt context in backend/app/services/rag_service.py
- [ ] T036 [US3] Create POST /api/chat/selected-text endpoint with selected_text and question validation in backend/app/api/chat_routes.py
- [ ] T037 [US3] Implement selected text flow: embed selection → inject into prompt → generate answer → save history
- [ ] T038 [US3] Update chat_messages schema to store selected_text (NULL for general queries)
- [ ] T039 [US3] Add validation: selected_text length (min 10 chars, max 5000 chars)

**Checkpoint**: At this point, selected text queries should work (test by selecting a paragraph and asking a question)

---

## Phase 6: User Story 4 - Chat History Management (Priority: P4)

**Goal**: Users can retrieve and clear their chat history

**Independent Test**: GET /api/chat/history/{session_id} returns all messages for that session in chronological order

### Implementation for User Story 4

- [ ] T040 [P] [US4] Create GET /api/chat/history/{session_id} endpoint in backend/app/api/chat_routes.py
- [ ] T041 [P] [US4] Implement chat history retrieval: query chat_messages table ordered by created_at in backend/app/services/db_service.py
- [ ] T042 [US4] Create POST /api/chat/clear endpoint with session_id validation in backend/app/api/chat_routes.py
- [ ] T043 [US4] Implement chat history deletion: DELETE FROM chat_messages WHERE session_id = ? in backend/app/services/db_service.py
- [ ] T044 [US4] Add pagination support for chat history (limit 50 messages per page, offset-based)

**Checkpoint**: At this point, chat history management should work (test retrieve and clear operations)

---

## Phase 7: User Story 5 - Frontend Chatbot Component (Priority: P5) 🎯 MVP

**Goal**: Users see a chatbot UI in Docusaurus that allows them to ask questions

**Independent Test**: Visit any chapter page, see chatbot button (bottom-right), click to open chat, send message, receive response

### Implementation for User Story 5

- [ ] T045 [P] [US5] Create ChatBot.tsx main component with open/close state in book-source/src/components/ChatBot/ChatBot.tsx
- [ ] T046 [P] [US5] Create ChatMessage.tsx for rendering user/assistant messages in book-source/src/components/ChatBot/ChatMessage.tsx
- [ ] T047 [P] [US5] Create ChatBot.module.css with floating button and chat window styles in book-source/src/components/ChatBot/ChatBot.module.css
- [ ] T048 [P] [US5] Define TypeScript interfaces for Message, ChatResponse in book-source/src/components/ChatBot/types.ts
- [ ] T049 [US5] Implement API client: fetch POST /api/chat with session management (UUID generation) in book-source/src/components/ChatBot/ChatBot.tsx
- [ ] T050 [US5] Implement message state management: user messages + assistant responses array in book-source/src/components/ChatBot/ChatBot.tsx
- [ ] T051 [US5] Implement chat UI: message input, send button, message list with scroll-to-bottom in book-source/src/components/ChatBot/ChatBot.tsx
- [ ] T052 [US5] Implement loading state: show "Thinking..." while waiting for API response
- [ ] T053 [US5] Implement error handling: display error message if API call fails
- [ ] T054 [US5] Create ChatBotButton.tsx floating button (bottom-right, sticky) in book-source/src/components/ChatBot/ChatBotButton.tsx
- [ ] T055 [US5] Integrate ChatBotButton into Docusaurus theme (add to layout or custom wrapper)
- [ ] T056 [P] [US5] Implement source citations display: clickable chapter/lesson links below each answer
- [ ] T057 [P] [US5] Add session persistence: store session_id in localStorage
- [ ] T058 [P] [US5] Implement "Clear History" button that calls POST /api/chat/clear

**Checkpoint**: At this point, chatbot UI should be fully functional (test on localhost)

---

## Phase 8: User Story 6 - Selected Text Integration (Priority: P6)

**Goal**: Users can select text on a lesson page and ask questions about it via the chatbot

**Independent Test**: Select text on a lesson page, open chatbot, see "Ask about selected text" option, send question, receive contextual answer

### Implementation for User Story 6

- [ ] T059 [P] [US6] Create SelectedTextChat.tsx component for selected text UI in book-source/src/components/ChatBot/SelectedTextChat.tsx
- [ ] T060 [US6] Implement text selection detection: listen for 'mouseup' event on lesson content
- [ ] T061 [US6] Implement selected text storage: save window.getSelection().toString() to state
- [ ] T062 [US6] Add "Ask about selected text" mode toggle in ChatBot.tsx
- [ ] T063 [US6] Implement API client for POST /api/chat/selected-text endpoint
- [ ] T064 [US6] Display selected text snippet in chat UI (with ellipsis if > 200 chars)
- [ ] T065 [US6] Add "Clear selection" button to exit selected text mode

**Checkpoint**: At this point, selected text queries should work from the UI

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T066 [P] Add comprehensive docstrings (Google style) to all backend services
- [ ] T067 [P] Add OpenAPI documentation tags to all FastAPI endpoints in backend/app/api/
- [ ] T068 [P] Implement request/response logging for debugging in backend/app/utils/logger.py
- [ ] T069 [P] Add input sanitization for user messages (prevent XSS) in backend/app/api/chat_routes.py
- [ ] T070 [P] Optimize Qdrant queries: add filters for chapter/lesson if available
- [ ] T071 [P] Add caching layer for frequently asked questions (Redis or in-memory LRU cache)
- [ ] T072 [P] Implement token usage tracking and cost monitoring for OpenAI API calls
- [ ] T073 [P] Add analytics: track query topics, response times, cache hit rate
- [ ] T074 [P] Frontend: Add keyboard shortcuts (Enter to send, Esc to close chatbot)
- [ ] T075 [P] Frontend: Add mobile responsive styles for chatbot UI
- [ ] T076 [P] Frontend: Implement markdown rendering for code blocks in assistant responses
- [ ] T077 [P] Create README.md with setup instructions in backend/README.md
- [ ] T078 [P] Create environment variables documentation in backend/.env.example (with example values)
- [ ] T079 Run full integration test: ingest embeddings → ask 10 sample questions → verify answers
- [ ] T080 Deploy backend to production (Vercel/Railway/Render) and test CORS with GitHub Pages frontend

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - **US1 (Embeddings)**: Can start after Foundational
  - **US2 (Chat)**: Depends on US1 (requires embeddings in Qdrant)
  - **US3 (Selected Text)**: Depends on US2 (extends chat functionality)
  - **US4 (History)**: Can start after Foundational (independent of US2/US3)
  - **US5 (Frontend)**: Depends on US2 and US4 (requires working chat + history APIs)
  - **US6 (Selected Text UI)**: Depends on US3 and US5 (requires selected text API + frontend)
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (Embeddings Ingestion)**: Can start after Foundational - No dependencies on other stories
- **US2 (General Chat)**: Depends on US1 completion (requires embeddings)
- **US3 (Selected Text Queries)**: Depends on US2 completion (extends chat logic)
- **US4 (Chat History)**: Can start after Foundational - Independent of US1/US2/US3
- **US5 (Frontend Chatbot)**: Depends on US2 and US4 completion (requires chat + history APIs)
- **US6 (Selected Text UI)**: Depends on US3 and US5 completion (requires selected text API + chatbot UI)

### Within Each User Story

- **US1**: MDX processing → chunking → embedding → ingestion → API endpoint
- **US2**: Vector search → RAG service → prompt engineering → API endpoint → error handling
- **US3**: Selected text embedding → query handler → API endpoint → validation
- **US4**: Database queries → API endpoints (history retrieval + clear)
- **US5**: React components → API client → state management → UI integration
- **US6**: Text selection detection → API client → UI mode toggle

### Parallel Opportunities

- **Phase 1**: T003 (env config), T004 (CORS), T005 (logging) can run in parallel
- **Phase 2**: T008 (Qdrant), T009 (OpenAI), T012 (schemas), T014 (collection), T015 (rate limiting) can run in parallel after T006-T007 complete
- **US1**: T016 (markdown), T017 (chunking), T018 (metadata) can run in parallel
- **US2**: T024 (vector search), T025 (OpenAI service) can run in parallel; T032-T033 (error handling) can run in parallel
- **US3**: T034 (embedding), T035 (query handler) can run in parallel
- **US4**: T040-T041 (history retrieval), T042-T043 (history clear) can run in parallel
- **US5**: T045 (ChatBot), T046 (ChatMessage), T047 (CSS), T048 (types) can run in parallel; T056 (citations), T057 (session), T058 (clear button) can run in parallel
- **US6**: T059 (SelectedTextChat), T060 (selection detection) can run in parallel
- **Phase 9**: Most polish tasks can run in parallel (T066-T078)

---

## Parallel Example: User Story 2 (General Chat)

```bash
# Launch parallel tasks for vector search and OpenAI service:
Task: "Implement vector search in backend/app/services/embeddings_service.py"
Task: "Create OpenAI service wrapper in backend/app/services/openai_service.py"

# After both complete, implement RAG service that uses both:
Task: "Implement RAG service in backend/app/services/rag_service.py"

# Launch parallel error handling tasks:
Task: "Add error handling for OpenAI API failures"
Task: "Add error handling for Qdrant timeouts"
```

---

## Implementation Strategy

### MVP First (US1 + US2 + US5 Only)

**Goal**: Get a working chatbot that can answer questions

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: US1 (Embeddings) → Verify with Qdrant dashboard
4. Complete Phase 4: US2 (General Chat) → Test with API client (Postman/curl)
5. Complete Phase 7: US5 (Frontend) → Test in browser
6. **STOP and VALIDATE**: Test end-to-end chatbot flow
7. Deploy/demo if ready

**MVP Scope**: US1 (ingest content) + US2 (chat queries) + US5 (UI) = **Functional chatbot** ✅

### Full Feature Set

After MVP validation:

8. Complete Phase 5: US3 (Selected Text API)
9. Complete Phase 6: US4 (Chat History)
10. Complete Phase 8: US6 (Selected Text UI)
11. Complete Phase 9: Polish & Cross-Cutting Concerns
12. Final integration testing and deployment

### Incremental Delivery Milestones

- **Milestone 1**: Setup + Foundational → Infrastructure ready
- **Milestone 2**: + US1 → Content embedded (backend-only, test with API)
- **Milestone 3**: + US2 → Chat queries work (backend-only, test with API)
- **Milestone 4**: + US5 → **MVP: Working chatbot UI** 🎯
- **Milestone 5**: + US3 + US6 → Selected text queries
- **Milestone 6**: + US4 → Chat history management
- **Milestone 7**: + Polish → **Production-ready**

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Focus on MVP first (US1 + US2 + US5) before adding advanced features
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Total Task Count

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 10 tasks
- **Phase 3 (US1 - Embeddings)**: 8 tasks
- **Phase 4 (US2 - General Chat)**: 10 tasks
- **Phase 5 (US3 - Selected Text)**: 6 tasks
- **Phase 6 (US4 - Chat History)**: 5 tasks
- **Phase 7 (US5 - Frontend)**: 14 tasks
- **Phase 8 (US6 - Selected Text UI)**: 7 tasks
- **Phase 9 (Polish)**: 15 tasks

**Total**: 80 tasks

**MVP Tasks** (US1 + US2 + US5): 5 (setup) + 10 (foundational) + 8 (US1) + 10 (US2) + 14 (US5) = **47 tasks**

**Parallel Opportunities**: 35+ tasks can run in parallel at various stages
