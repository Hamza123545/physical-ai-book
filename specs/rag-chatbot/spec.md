# Spec: RAG Chatbot Backend

**Created Using**: SpecKit Plus + Claude Code  
**Workflow**: `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`  
**Status**: ✅ Complete  
**Date**: 2025-01-29  
**Implemented By**: Claude (Anthropic) + SpecKit Plus  
**Completion Date**: 2025-12-01

---

## Feature Overview

**Feature Name**: RAG Chatbot Backend  
**Priority**: P1 (Base - 100 points)  
**Type**: Backend API Service

### Description

Build a Retrieval-Augmented Generation (RAG) chatbot backend that answers questions about the Physical AI & Humanoid Robotics textbook content. The chatbot uses OpenAI Agents SDK with support for both OpenAI GPT-4 and Gemini models via LiteLLM.

### Requirements

1. **Embed all book content** (`book-source/docs/*.md`) into Qdrant vector database
2. **Answer general questions** about textbook content using RAG
3. **Answer selected text queries** - User selects text → Ask question → Answer based only on selected text
4. **Store chat history** in Neon Postgres database
5. **Support multiple LLM models** - OpenAI GPT-4 and Gemini (configurable)
6. **React chatbot component** embedded in Docusaurus frontend

---

## User Stories

### US1: Content Ingestion (P1)

**As a** system administrator  
**I want to** ingest all textbook content into Qdrant  
**So that** the chatbot can retrieve relevant information

**Acceptance Criteria**:
- All markdown files from `book-source/docs/*.md` are processed
- Content is chunked (512 tokens, 100 overlap)
- Embeddings generated using OpenAI `text-embedding-3-small`
- Vectors stored in Qdrant with metadata (chapter, lesson, file_path)
- API endpoint: `POST /api/embeddings/ingest`

### US2: General Chat Query (P1)

**As a** student  
**I want to** ask general questions about the textbook  
**So that** I can get answers based on the book content

**Acceptance Criteria**:
- User sends question via `POST /api/chat`
- System retrieves top 5 relevant chunks from Qdrant
- Response generated using OpenAI Agents SDK (Agent + Runner)
- Supports both OpenAI GPT-4 and Gemini models
- Response includes source citations
- Chat history stored in Postgres

### US3: Selected Text Query (P1)

**As a** student  
**I want to** ask questions about specific selected text  
**So that** I get answers based only on that text

**Acceptance Criteria**:
- User selects text and asks question via `POST /api/chat/selected-text`
- System finds similar chunks in Qdrant (limited to selected text context)
- Response generated using OpenAI Agents SDK
- Answer based only on selected text context
- Source citations included

### US4: Chat History Management (P2)

**As a** student  
**I want to** view and manage my chat history  
**So that** I can review previous conversations

**Acceptance Criteria**:
- `GET /api/chat/history/{session_id}` - Retrieve chat history
- `POST /api/chat/clear` - Clear chat history
- History stored in Postgres with session management
- Messages linked to sessions

### US5: Frontend Integration (P1)

**As a** student  
**I want to** use the chatbot in the Docusaurus frontend  
**So that** I can ask questions while reading

**Acceptance Criteria**:
- React chatbot component (`Chatbot.tsx`)
- Chat UI with messages, input, send button
- Selected text detection (highlight → "Ask about this" button)
- Connect to FastAPI endpoints
- Display streaming responses (optional)
- Show chat history

---

## Technical Stack

- **Framework**: FastAPI (Python 3.11+)
- **Database**: Neon Postgres (SQLAlchemy, Alembic)
- **Vector DB**: Qdrant Cloud
- **LLM**: OpenAI Agents SDK (https://openai.github.io/openai-agents-python/)
- **Models**: OpenAI GPT-4 or Gemini (via LiteLLM)
- **Embeddings**: OpenAI `text-embedding-3-small`
- **Frontend**: React (Docusaurus)

---

## API Endpoints

### Health & Info
- `GET /` - Service status
- `GET /health` - Detailed health check

### Embeddings
- `POST /api/embeddings/ingest` - Ingest book content into Qdrant

### Chat
- `POST /api/chat` - General chat query
- `POST /api/chat/selected-text` - Selected text query
- `GET /api/chat/history/{session_id}` - Get chat history
- `POST /api/chat/clear` - Clear chat history

---

## Data Models

### ChatSession
- `id` (UUID, primary key)
- `created_at` (timestamp)
- `updated_at` (timestamp)

### ChatMessage
- `id` (UUID, primary key)
- `session_id` (UUID, foreign key)
- `role` (enum: user, assistant)
- `content` (text)
- `selected_text` (text, nullable)
- `retrieved_chunks` (JSON, nullable)
- `created_at` (timestamp)

### SourceCitation
- `chapter` (string)
- `lesson` (string)
- `section_title` (string)
- `file_path` (string)
- `similarity_score` (float)

---

## Environment Variables

```env
# Database
DATABASE_URL=postgresql://user:pass@host:5432/dbname

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI (for embeddings)
OPENAI_API_KEY=sk-your-openai-key

# Model Selection
USE_GEMINI=false  # true for Gemini, false for OpenAI
CHAT_MODEL=gpt-4-turbo-preview  # or gemini/gemini-2.0-flash-exp
GEMINI_API_KEY=your-gemini-key  # Required if USE_GEMINI=true

# App Config
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,https://your-site.github.io
```

---

## Success Criteria

✅ All book content embedded in Qdrant  
✅ General chat queries return accurate answers with citations  
✅ Selected text queries work correctly  
✅ Chat history stored and retrievable  
✅ Frontend chatbot component functional  
✅ Supports both OpenAI and Gemini models  
✅ All API endpoints tested and documented

---

## References

- OpenAI Agents SDK: https://openai.github.io/openai-agents-python/
- Constitution: `.specify/memory/constitution.md` (Backend Development Principles)
- Qdrant Docs: https://qdrant.tech/documentation/
- FastAPI Docs: https://fastapi.tiangolo.com/

