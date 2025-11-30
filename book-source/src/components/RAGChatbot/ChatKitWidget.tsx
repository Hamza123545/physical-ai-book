/**
 * OpenAI ChatKit Widget for RAG Chatbot
 * Official ChatKit React integration for Physical AI Textbook
 *
 * NOTE: Using HostedApiConfig with getClientSecret for simpler integration.
 * The backend only needs to implement a session creation endpoint.
 */

import React from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import styles from './chatkit.module.css';

export default function ChatKitWidget() {
  const { control } = useChatKit({
    // HostedApiConfig - Simple authentication via client secret
    api: {
      async getClientSecret(existing) {
        console.log('[ChatKit] Requesting client secret', { existing });

        try {
          // Use production backend URL or localhost for development
          const apiUrl = (typeof window !== 'undefined' && 
            (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'))
            ? 'http://localhost:8000'
            : 'https://physical-ai-backend-9lxv.onrender.com';
          
          const response = await fetch(`${apiUrl}/api/chatkit/session`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              existing_session: existing || null,
            }),
          });

          if (!response.ok) {
            throw new Error(`Session creation failed: ${response.statusText}`);
          }

          const data = await response.json();
          console.log('[ChatKit] Client secret received');
          return data.client_secret;
        } catch (error) {
          console.error('[ChatKit] Failed to get client secret:', error);
          throw error;
        }
      },
    },

    // Theme configuration
    theme: 'light', // Can be 'light', 'dark', or a custom theme object

    // Initial thread
    initialThread: null, // Start with new thread

    // Header configuration
    header: {
      enabled: true,
      title: {
        enabled: true,
        text: 'Physical AI Assistant',
      },
    },

    // History panel
    history: {
      enabled: true,
      showDelete: true,
      showRename: true,
    },

    // Start screen configuration
    startScreen: {
      enabled: true,
      title: 'Physical AI Assistant',
      subtitle: 'Powered by RAG & OpenAI Agents SDK',
      suggestions: [
        'What is physical AI?',
        'How do humanoid robots work?',
        'Explain reinforcement learning',
        'What are the key sensors in robots?',
      ],
    },

    // Composer configuration
    composer: {
      enabled: true,
      placeholder: 'Ask about physical AI...',
      attachments: {
        enabled: false, // Disable file uploads
      },
    },

    // Disclaimer
    disclaimer: {
      enabled: true,
      text: 'This AI assistant provides information about Physical AI and Humanoid Robotics. Always verify critical information.',
    },
  });

  return (
    <div className={styles.chatKitWrapper}>
      <ChatKit control={control} />
    </div>
  );
}
