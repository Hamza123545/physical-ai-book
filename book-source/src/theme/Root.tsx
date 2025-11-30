/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 2. RAGChatbot - Custom RAG chatbot with floating button (works WITHOUT workflow)
 *
 * AnalyticsTracker removed - not needed for hackathon
 */

import React from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
import RAGChatbot from '@/components/RAGChatbot';
// AnalyticsTracker removed - not needed for hackathon

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <PyodideProvider>
      {children}
      <RAGChatbot />
    </PyodideProvider>
  );
}
