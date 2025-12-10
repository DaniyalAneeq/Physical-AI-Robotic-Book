import { useState, useEffect, useCallback } from 'react';

interface TextSelection {
  text: string;
  startOffset: number;
  endOffset: number;
}

interface UseTextSelectionReturn {
  selectedText: string | null;
  hasSelection: boolean;
  clearSelection: () => void;
}

export function useTextSelection(): UseTextSelectionReturn {
  const [selectedText, setSelectedText] = useState<string | null>(null);

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();

    if (!selection || selection.isCollapsed) {
      // No selection or collapsed selection
      return;
    }

    const text = selection.toString().trim();

    // Only capture meaningful selections (at least 10 characters)
    if (text.length >= 10) {
      // Check if selection is within the document content area
      const anchorNode = selection.anchorNode;
      if (anchorNode) {
        const isInContent = anchorNode.parentElement?.closest('.theme-doc-markdown');
        if (isInContent) {
          setSelectedText(text);
        }
      }
    }
  }, []);

  const clearSelection = useCallback(() => {
    setSelectedText(null);
    window.getSelection()?.removeAllRanges();
  }, []);

  useEffect(() => {
    // Listen for mouseup to capture selection
    const handleMouseUp = () => {
      // Small delay to ensure selection is complete
      setTimeout(handleSelectionChange, 10);
    };

    // Listen for keyboard selection (Shift+Arrow keys)
    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.shiftKey) {
        setTimeout(handleSelectionChange, 10);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('keyup', handleKeyUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleSelectionChange]);

  // Clear selection when it becomes empty
  useEffect(() => {
    const handleSelectionClear = () => {
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) {
        // Don't clear if user is interacting with chatbot
        const activeElement = document.activeElement;
        const isInChatbot = activeElement?.closest('[role="dialog"]');
        if (!isInChatbot) {
          setSelectedText(null);
        }
      }
    };

    document.addEventListener('selectionchange', handleSelectionClear);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionClear);
    };
  }, []);

  return {
    selectedText,
    hasSelection: selectedText !== null && selectedText.length > 0,
    clearSelection,
  };
}
