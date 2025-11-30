/**
 * RAG Chatbot Component using ChatKit SDK
 * Integrates OpenAI ChatKit React with RAG backend
 * 
 * ChatKit SDK handles all API calls from frontend.
 * Backend only provides session token (client_secret).
 */

import React, { useState, useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import styles from './styles.module.css';

// Get API base URL (same logic as chatApi.ts)
const getApiBaseUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }
  
  if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }
  
  return 'http://localhost:8000'; // TODO: Update with production backend URL
};

const API_BASE_URL = getApiBaseUrl();

export default function ChatKitRAG() {
  const [error, setError] = useState<string | null>(null);

  const { control } = useChatKit({
    api: {
      async getClientSecret(existing?: string): Promise<string> {
        try {
          // Get client secret from backend
          const res = await fetch(`${API_BASE_URL}/api/chatkit/session`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              existing_session: existing || undefined,
            }),
          });

          if (!res.ok) {
            throw new Error(`Failed to get session: ${res.statusText}`);
          }

          const data = await res.json();
          return data.client_secret;
        } catch (err) {
          console.error('ChatKit session error:', err);
          setError('Failed to initialize chat. Please check if the backend is running.');
          throw err;
        }
      },
    },
  });

  // Handle errors
  useEffect(() => {
    if (error) {
      console.error('ChatKit error:', error);
    }
  }, [error]);

  return (
    <div className={styles.chatkitContainer}>
      {error && (
        <div className={styles.errorBanner}>
          ⚠️ {error}
        </div>
      )}
      <ChatKit 
        control={control} 
        className={styles.chatkitComponent}
      />
    </div>
  );
}

