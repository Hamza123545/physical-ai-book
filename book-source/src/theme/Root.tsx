/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. AuthProvider - Global authentication state management
 * 2. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 3. RAGChatbot - Custom RAG chatbot with floating button (works WITHOUT workflow)
 *
 * AnalyticsTracker removed - not needed for hackathon
 */

import React from 'react';
import { AuthProvider } from '@/contexts/AuthContext';
import { PyodideProvider } from '@/contexts/PyodideContext';
import RAGChatbot from '@/components/RAGChatbot';
// AnalyticsTracker removed - not needed for hackathon

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      <PyodideProvider>
        {children}
        <RAGChatbot />
      </PyodideProvider>
    </AuthProvider>
  );
}
