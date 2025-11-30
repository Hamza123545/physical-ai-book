---
id: 0001
title: RAG Chatbot Backend Specification
stage: spec
date: 2025-01-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: main
user: system
command: /sp.specify
labels: ["rag-chatbot", "backend", "fastapi", "openai-agents-sdk", "qdrant", "neon-postgres", "gemini", "litellm", "phase-2", "base-100-points"]
links:
  spec: specs/rag-chatbot/spec.md
  ticket: null
  adr: specs/rag-chatbot/spec.md#technical-stack
  pr: null
files:
  - specs/rag-chatbot/spec.md
tests:
  - None (specification phase)
---

## Prompt

Create specification for RAG Chatbot Backend (Base - 100 points).

**Context**: Physical AI textbook backend (FastAPI, Neon Postgres, Qdrant Cloud, OpenAI Agents SDK)

**Requirements**:
- Embed all book content (book-source/docs/*.md) into Qdrant
- Support general queries + selected text queries
- API: POST /api/chat, POST /api/chat/selected-text, GET /api/chat/history, POST /api/chat/clear, POST /api/embeddings/ingest
- React chatbot component in Docusaurus frontend
- Store chat history in Postgres
- Support both OpenAI GPT-4 and Gemini models (via LiteLLM)

**Constitution**: Follow .specify/memory/constitution.md (Backend Development Principles)

**Output**: Complete specification with user stories, technical stack, API endpoints, data models, environment variables, success criteria.

## Response snapshot

Created comprehensive specification (`specs/rag-chatbot/spec.md`) for RAG Chatbot Backend following constitution-mandated backend development workflow.

**Key Outputs**:

1. **Feature Overview** - RAG chatbot backend with OpenAI Agents SDK integration
2. **User Stories** - 5 user stories (US1: Content Ingestion, US2: General Chat, US3: Selected Text Query, US4: Chat History, US5: Frontend Integration)
3. **Technical Stack** - FastAPI, Neon Postgres, Qdrant Cloud, OpenAI Agents SDK, LiteLLM (Gemini support), OpenAI embeddings
4. **API Endpoints** - Complete REST API specification (health, embeddings, chat endpoints)
5. **Data Models** - ChatSession, ChatMessage, SourceCitation schemas
6. **Environment Variables** - Complete configuration (database, Qdrant, OpenAI, Gemini)
7. **Success Criteria** - Measurable acceptance criteria for all features

**Constitution Compliance**: ✅ All Backend Development Principles followed (FastAPI-first, async, security, error handling, code organization)

