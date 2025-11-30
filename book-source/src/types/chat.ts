/**
 * TypeScript types for RAG Chatbot
 * Matches backend API schemas from backend/app/models/schemas.py
 */

export interface SourceCitation {
  chapter: string;
  lesson: string;
  section_title: string;
  file_path: string;
  similarity_score: number;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SourceCitation[];
}

export interface ChatRequest {
  session_id: string;
  message: string;
}

export interface ChatResponse {
  session_id: string;
  message: string;
  sources: SourceCitation[];
  timestamp: string;
}

export interface ChatHistoryResponse {
  session_id: string;
  messages: ChatMessage[];
  created_at: string;
  updated_at: string;
}

export interface SelectedTextChatRequest {
  session_id: string;
  message: string;
  selected_text: string;
}

export interface ClearHistoryRequest {
  session_id?: string;
}

export interface ClearHistoryResponse {
  success: boolean;
  message: string;
  deleted_sessions: number;
  deleted_messages: number;
}
