/**
 * API service for RAG Chatbot backend communication
 * Connects to FastAPI backend at localhost:8000
 */

import type {
  ChatRequest,
  ChatResponse,
  ChatHistoryResponse,
  SelectedTextChatRequest,
  ClearHistoryRequest,
  ClearHistoryResponse,
} from '../types/chat';

// Backend API base URL
// In browser, we detect environment based on hostname
// Development: localhost -> use local backend
// Production: deployed site -> use production backend URL
const getApiBaseUrl = (): string => {
  if (typeof window === 'undefined') {
    // Server-side rendering - default to localhost
    return 'http://localhost:8000';
  }
  
  // Check if we're in development (localhost)
  if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }
  
  // Production: Use Render backend URL
  return 'https://physical-ai-backend-9lxv.onrender.com';
};

const API_BASE_URL = getApiBaseUrl();

/**
 * Send a chat message and get RAG-enhanced response
 */
export async function sendChatMessage(
  sessionId: string,
  message: string
): Promise<ChatResponse> {
  const request: ChatRequest = {
    session_id: sessionId,
    message,
  };

  const response = await fetch(`${API_BASE_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`Chat API error: ${response.status} ${response.statusText}`);
  }

  return response.json();
}

/**
 * Send a question about selected text
 */
export async function sendSelectedTextQuery(
  sessionId: string,
  message: string,
  selectedText: string
): Promise<ChatResponse> {
  const request: SelectedTextChatRequest = {
    session_id: sessionId,
    message,
    selected_text: selectedText,
  };

  const response = await fetch(`${API_BASE_URL}/api/chat/selected-text`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`Selected text API error: ${response.status} ${response.statusText}`);
  }

  return response.json();
}

/**
 * Get chat history for a session
 */
export async function getChatHistory(sessionId: string): Promise<ChatHistoryResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chat/history/${sessionId}`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    throw new Error(`History API error: ${response.status} ${response.statusText}`);
  }

  return response.json();
}

/**
 * Clear chat history for a session or all sessions
 */
export async function clearChatHistory(sessionId?: string): Promise<ClearHistoryResponse> {
  const request: ClearHistoryRequest = {
    session_id: sessionId,
  };

  const response = await fetch(`${API_BASE_URL}/api/chat/clear`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`Clear history API error: ${response.status} ${response.statusText}`);
  }

  return response.json();
}

/**
 * Generate a unique session ID
 */
export function generateSessionId(): string {
  return `session-${Date.now()}-${Math.random().toString(36).substring(2, 9)}`;
}

/**
 * Get or create session ID from localStorage
 */
export function getSessionId(): string {
  const storageKey = 'rag-chatbot-session-id';

  // Try to get existing session ID
  if (typeof window !== 'undefined') {
    const existingId = localStorage.getItem(storageKey);
    if (existingId) {
      return existingId;
    }
  }

  // Generate new session ID
  const newId = generateSessionId();

  // Save to localStorage
  if (typeof window !== 'undefined') {
    localStorage.setItem(storageKey, newId);
  }

  return newId;
}

/**
 * Clear session ID (useful for starting fresh conversation)
 */
export function clearSessionId(): void {
  if (typeof window !== 'undefined') {
    localStorage.removeItem('rag-chatbot-session-id');
  }
}
