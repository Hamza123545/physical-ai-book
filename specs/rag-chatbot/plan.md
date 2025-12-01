# Plan: RAG Chatbot Backend

**Created Using**: SpecKit Plus + Claude Code  
**Workflow**: `/sp.plan` (after `/sp.specify`)  
**Input**: `specs/rag-chatbot/spec.md`  
**Status**: ✅ Complete  
**Date**: 2025-01-29  
**Implemented By**: Claude (Anthropic) + SpecKit Plus  
**Completion Date**: 2025-12-01

---

## Implementation Strategy

### Phase-Driven Approach

**Phase 1**: Infrastructure Setup (Foundation)  
**Phase 2**: Content Ingestion (Data Pipeline)  
**Phase 3**: RAG Core Logic (OpenAI Agents SDK Integration)  
**Phase 4**: Selected Text Query (Context Limiting)  
**Phase 5**: API Endpoints (REST API)  
**Phase 6**: Frontend Integration (React Component)  
**Phase 7**: Testing & Documentation (Quality Assurance)

---

## Phase 1: Infrastructure Setup

**Purpose**: Initialize FastAPI project, database, and external service connections

**Tasks**:
1. Initialize FastAPI project structure in `backend/`
2. Create `backend/app/config.py` with environment variables
3. Setup Neon Postgres connection (asyncpg, connection pooling)
4. Setup Qdrant client and connection
5. Create database models (SQLAlchemy): `ChatSession`, `ChatMessage`
6. Create Alembic migrations
7. Write `requirements.txt` with all dependencies
8. Create `.env.example` with all required variables

**Files to Create**:
- `backend/app/main.py`
- `backend/app/config.py`
- `backend/app/models/chat_history.py`
- `backend/app/models/schemas.py`
- `backend/requirements.txt`
- `backend/.env.example`
- `backend/alembic.ini`
- `backend/alembic/env.py`

**Dependencies**: None (foundation phase)

---

## Phase 2: Content Ingestion

**Purpose**: Process and embed all textbook content into Qdrant

**Tasks**:
1. Read all `book-source/docs/*.md` files
2. Parse markdown and extract content
3. Split into chunks (512 tokens, 100 token overlap)
4. Generate embeddings using OpenAI `text-embedding-3-small`
5. Store in Qdrant collection "physical_ai_textbook"
6. Add metadata (chapter, lesson, file_path, section_title)
7. Create `POST /api/embeddings/ingest` endpoint
8. Add batch processing for large content

**Files to Create**:
- `backend/app/services/embeddings_service.py`
- `backend/app/services/markdown_processor.py`
- `backend/app/api/embeddings_routes.py`

**Dependencies**: Phase 1 (Infrastructure)

---

## Phase 3: RAG Core Logic

**Purpose**: Implement RAG using OpenAI Agents SDK with multi-model support

**Tasks**:
1. Install and configure OpenAI Agents SDK
2. Install LiteLLM for Gemini support
3. Create `agent_service.py` with Agent creation functions
4. Implement `create_rag_agent()` - Agent with RAG context
5. Implement `create_selected_text_agent()` - Agent for selected text
6. Implement `run_agent_sync()` - Synchronous agent execution
7. Implement `run_agent_stream()` - Streaming agent execution (optional)
8. Create RAG service that orchestrates: Qdrant search → Agent → Response
9. Extract source citations from retrieved chunks
10. Support both OpenAI GPT-4 and Gemini models (configurable)

**Files to Create**:
- `backend/app/services/agent_service.py`
- `backend/app/services/rag_service.py`

**Files to Update**:
- `backend/app/config.py` (add Gemini config)
- `backend/requirements.txt` (add `openai-agents`, `litellm`, `google-generativeai`)

**Dependencies**: Phase 1 (Infrastructure), Phase 2 (Embeddings)

---

## Phase 4: Selected Text Query

**Purpose**: Support queries based on user-selected text

**Tasks**:
1. Create `POST /api/chat/selected-text` endpoint
2. Accept `selected_text` and `question` in request
3. Generate embedding for selected text
4. Find similar chunks in Qdrant (limit to 3, higher threshold 0.8)
5. Create agent with selected text context
6. Generate answer using only selected text context
7. Return response with citations

**Files to Update**:
- `backend/app/api/chat_routes.py` (add selected-text endpoint)
- `backend/app/services/rag_service.py` (add `generate_selected_text_response()`)

**Dependencies**: Phase 3 (RAG Core)

---

## Phase 5: API Endpoints

**Purpose**: Complete REST API with all endpoints

**Tasks**:
1. Create `POST /api/chat` endpoint (general query)
2. Create `POST /api/chat/selected-text` endpoint
3. Create `GET /api/chat/history/{session_id}` endpoint
4. Create `POST /api/chat/clear` endpoint
5. Add error handling and validation (Pydantic models)
6. Add CORS configuration for Docusaurus frontend
7. Add rate limiting middleware
8. Add request/response logging
9. Register all routes in `main.py`
10. Add OpenAPI/Swagger documentation

**Files to Create/Update**:
- `backend/app/api/chat_routes.py`
- `backend/app/services/db_service.py`
- `backend/app/main.py` (register routes, add middleware)

**Dependencies**: Phase 1 (Infrastructure), Phase 3 (RAG Core), Phase 4 (Selected Text)

---

## Phase 6: Frontend Integration

**Purpose**: React chatbot component for Docusaurus

**Tasks**:
1. Create React component `book-source/src/components/Chatbot.tsx`
2. Design chat UI (messages list, input field, send button)
3. Implement selected text detection (highlight text → show "Ask about this" button)
4. Connect to FastAPI endpoints (fetch API)
5. Display streaming responses (optional - polling or WebSocket)
6. Show chat history in component
7. Add loading states and error handling
8. Style with CSS module `Chatbot.module.css`
9. Integrate into Docusaurus pages (optional)

**Files to Create**:
- `book-source/src/components/Chatbot.tsx`
- `book-source/src/components/Chatbot.module.css`

**Dependencies**: Phase 5 (API Endpoints)

---

## Phase 7: Testing & Documentation

**Purpose**: Quality assurance and documentation

**Tasks**:
1. Write unit tests for services (mock OpenAI, Qdrant, Postgres)
2. Write integration tests for API endpoints
3. Test selected text query functionality
4. Test model switching (OpenAI ↔ Gemini)
5. Update `backend/README.md` with setup instructions
6. Add API documentation (FastAPI Swagger)
7. Create `GEMINI_SETUP.md` guide
8. Document environment variables
9. Add troubleshooting section

**Files to Create**:
- `backend/tests/test_rag_service.py`
- `backend/tests/test_chat_routes.py`
- `backend/tests/test_agent_service.py`
- `backend/GEMINI_SETUP.md`

**Files to Update**:
- `backend/README.md`

**Dependencies**: All previous phases

---

## File Structure

```
backend/
├── app/
│   ├── main.py                    # FastAPI app entry point
│   ├── config.py                  # Configuration (env vars, clients)
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat_routes.py         # Chat API endpoints
│   │   └── embeddings_routes.py   # Embeddings ingestion endpoint
│   ├── models/
│   │   ├── __init__.py
│   │   ├── chat_history.py         # SQLAlchemy models
│   │   └── schemas.py             # Pydantic request/response schemas
│   ├── services/
│   │   ├── __init__.py
│   │   ├── agent_service.py       # OpenAI Agents SDK integration
│   │   ├── rag_service.py         # RAG orchestration
│   │   ├── embeddings_service.py  # Qdrant operations
│   │   ├── db_service.py          # Database operations
│   │   └── markdown_processor.py # MDX parsing & chunking
│   └── utils/
│       ├── __init__.py
│       └── logger.py              # Structured logging
├── alembic/                        # Database migrations
│   ├── versions/
│   └── env.py
├── tests/                         # Test files
├── requirements.txt
├── .env.example
├── README.md
└── GEMINI_SETUP.md
```

---

## Dependencies Between Phases

```
Phase 1 (Infrastructure)
    ↓
Phase 2 (Content Ingestion) ──┐
    ↓                          │
Phase 3 (RAG Core) ────────────┘
    ↓
Phase 4 (Selected Text)
    ↓
Phase 5 (API Endpoints)
    ↓
Phase 6 (Frontend)
    ↓
Phase 7 (Testing)
```

---

## Estimated Time

- **Phase 1**: 2-3 hours
- **Phase 2**: 3-4 hours
- **Phase 3**: 4-5 hours
- **Phase 4**: 2-3 hours
- **Phase 5**: 3-4 hours
- **Phase 6**: 4-5 hours
- **Phase 7**: 3-4 hours

**Total**: ~21-28 hours

---

## Success Metrics

✅ All 7 phases completed  
✅ All API endpoints functional  
✅ Frontend chatbot component working  
✅ Both OpenAI and Gemini models supported  
✅ Chat history stored and retrievable  
✅ Selected text queries working  
✅ Tests passing  
✅ Documentation complete

