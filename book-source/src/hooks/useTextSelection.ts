/**
 * Custom hook for detecting text selection and showing chatbot UI
 */

import { useState, useEffect, useCallback } from 'react';

interface TextSelection {
  text: string;
  position: { x: number; y: number };
}

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
    const selection = window.getSelection();
    
    if (!selection || selection.rangeCount === 0) {
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

