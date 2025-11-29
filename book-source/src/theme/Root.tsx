/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 *
 * AnalyticsTracker removed - not needed for hackathon
 */

import React from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
// AnalyticsTracker removed - not needed for hackathon

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <PyodideProvider>
      {children}
    </PyodideProvider>
  );
}
