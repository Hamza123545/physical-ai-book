/**
 * Custom hook for detecting text selection and showing chatbot UI
 * Only works on book content pages (docs)
 */

import { useState, useEffect, useCallback } from 'react';

interface TextSelection {
  text: string;
  position: { x: number; y: number };
}

// Check if we're on a book/docs page
const isBookPage = (): boolean => {
  if (typeof window === 'undefined') return false;
  
  const path = window.location.pathname;
  
  // Only enable on docs/book pages
  // Exclude: login, signup, and other non-book pages
  const excludedPaths = ['/login', '/signup', '/profile', '/settings'];
  
  for (const excluded of excludedPaths) {
    if (path.includes(excluded)) {
      return false;
    }
  }
  
  // Must be on a docs page (contains /docs/ or chapter content)
  return path.includes('/docs/') || 
         path.includes('/chapter-') || 
         path.includes('/intro') ||
         path.includes('/appendix');
};

// Check if selection is within book content
const isSelectionInBookContent = (selection: Selection): boolean => {
  if (!selection.rangeCount) return false;
  
  const range = selection.getRangeAt(0);
  const container = range.commonAncestorContainer;
  
  // Get the element
  const element = container.nodeType === Node.TEXT_NODE 
    ? container.parentElement 
    : container as Element;
  
  if (!element) return false;
  
  // Check if inside article/main content area
  const articleContent = element.closest('article, .theme-doc-markdown, [class*="docItemContainer"], main');
  
  // Exclude navbar, footer, sidebar
  const isInExcludedArea = element.closest('nav, footer, .navbar, .sidebar, form, input, button, [class*="sidebar"]');
  
  return !!articleContent && !isInExcludedArea;
};

export function useTextSelection(
  onSelection: (text: string) => void
): {
  selectedText: string | null;
  selectionPosition: { x: number; y: number } | null;
  clearSelection: () => void;
} {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [selectionPosition, setSelectionPosition] = useState<{ x: number; y: number } | null>(null);

  const handleSelection = useCallback(() => {
    // Only work on book pages
    if (!isBookPage()) {
      setSelectedText(null);
      setSelectionPosition(null);
      return;
    }

    const selection = window.getSelection();
    
    if (!selection || selection.rangeCount === 0) {
      setSelectedText(null);
      setSelectionPosition(null);
      return;
    }

    // Check if selection is in book content
    if (!isSelectionInBookContent(selection)) {
      setSelectedText(null);
      setSelectionPosition(null);
      return;
    }

    const text = selection.toString().trim();
    
    // Only show if meaningful text is selected (more than 3 characters)
    if (text.length < 3) {
      setSelectedText(null);
      setSelectionPosition(null);
      return;
    }

    // Get position of selection
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();
    
    setSelectedText(text);
    setSelectionPosition({
      x: rect.left + rect.width / 2,
      y: rect.top - 10, // Above the selection
    });

    // Call callback with selected text
    onSelection(text);
  }, [onSelection]);

  useEffect(() => {
    // Listen for selection changes
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [handleSelection]);

  const clearSelection = useCallback(() => {
    setSelectedText(null);
    setSelectionPosition(null);
    // Clear browser selection
    if (window.getSelection) {
      window.getSelection()?.removeAllRanges();
    }
  }, []);

  return {
    selectedText,
    selectionPosition,
    clearSelection,
  };
}
