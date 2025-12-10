import { useState, useCallback, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import {
  sendChatMessage,
  createSession,
  getSession,
  getStoredSessionId,
  storeSessionId,
  clearStoredSessionId,
  type ScopeContext,
  type Citation,
  type StreamEvent,
} from '../services/chatApi';

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  isStreaming?: boolean;
}

interface UseChatbotReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
  currentScope: ScopeContext;
  chapterId: string | null;
  moduleId: string | null;
  sendMessage: (content: string) => Promise<void>;
  clearError: () => void;
  clearMessages: () => void;
  setScope: (scope: ScopeContext) => void;
  setScopeWithSelection: (selectedText: string) => void;
}

/**
 * Extract chapter_id from the current URL path.
 * Expected format: /docs/module-X/chapter-Y or similar
 */
function extractChapterIdFromPath(pathname: string): string | null {
  // Match patterns like /docs/module-1/chapter-1 or /docs/module-1/01-intro
  const match = pathname.match(/\/docs\/([^/]+)\/([^/]+)/);
  if (match) {
    return match[2].replace(/\.html?$/, '');
  }
  return null;
}

/**
 * Extract module_id from the current URL path.
 */
function extractModuleIdFromPath(pathname: string): string | null {
  const match = pathname.match(/\/docs\/([^/]+)/);
  if (match) {
    return match[1];
  }
  return null;
}

export function useChatbot(): UseChatbotReturn {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [currentScope, setCurrentScope] = useState<ScopeContext>({ type: 'all' });
  const [chapterId, setChapterId] = useState<string | null>(null);
  const [moduleId, setModuleId] = useState<string | null>(null);

  const location = useLocation();

  // Extract chapter and module IDs from URL
  useEffect(() => {
    const chapter = extractChapterIdFromPath(location.pathname);
    const module = extractModuleIdFromPath(location.pathname);
    setChapterId(chapter);
    setModuleId(module);

    // Update scope based on current page (default to page scope if available)
    if (chapter) {
      setCurrentScope({
        type: 'page',
        chapter_id: chapter,
      });
    } else {
      setCurrentScope({ type: 'all' });
    }
  }, [location.pathname]);

  // Initialize session from localStorage
  useEffect(() => {
    const initSession = async () => {
      const storedId = getStoredSessionId();
      if (storedId) {
        try {
          const session = await getSession(storedId);
          if (session) {
            setSessionId(storedId);
            return;
          }
        } catch (e) {
          // Session expired or invalid
          clearStoredSessionId();
        }
      }
    };
    initSession();
  }, []);

  const ensureSession = useCallback(async (): Promise<string> => {
    if (sessionId) {
      // Verify session still exists
      try {
        const session = await getSession(sessionId);
        if (session) {
          return sessionId;
        }
      } catch (e) {
        // Session expired
      }
    }

    // Create new session
    const newSession = await createSession(currentScope);
    setSessionId(newSession.id);
    storeSessionId(newSession.id);
    return newSession.id;
  }, [sessionId, currentScope]);

  const sendMessage = useCallback(async (content: string) => {
    if (!content.trim() || isLoading) return;

    setIsLoading(true);
    setError(null);

    // Add user message
    const userMessage: Message = { role: 'user', content };
    setMessages((prev) => [...prev, userMessage]);

    try {
      const activeSessionId = await ensureSession();

      // Add placeholder assistant message for streaming
      const assistantMessage: Message = {
        role: 'assistant',
        content: '',
        citations: [],
        isStreaming: true,
      };
      setMessages((prev) => [...prev, assistantMessage]);

      const citations: Citation[] = [];

      await sendChatMessage(
        {
          session_id: activeSessionId,
          query: content,
          scope: currentScope,
        },
        (event: StreamEvent) => {
          if (event.type === 'content' && event.text) {
            setMessages((prev) => {
              const updated = [...prev];
              const lastMsg = updated[updated.length - 1];
              if (lastMsg?.role === 'assistant') {
                updated[updated.length - 1] = {
                  ...lastMsg,
                  content: lastMsg.content + event.text,
                };
              }
              return updated;
            });
          } else if (event.type === 'citation' && event.citation) {
            citations.push(event.citation);
          } else if (event.type === 'done') {
            setMessages((prev) => {
              const updated = [...prev];
              const lastMsg = updated[updated.length - 1];
              if (lastMsg?.role === 'assistant') {
                updated[updated.length - 1] = {
                  ...lastMsg,
                  isStreaming: false,
                  citations,
                };
              }
              return updated;
            });
          } else if (event.type === 'error') {
            setError(event.error || 'An error occurred');
          }
        },
      );
    } catch (e) {
      const errorMessage = e instanceof Error ? e.message : 'Failed to send message';
      setError(errorMessage);

      // Remove the streaming assistant message on error
      setMessages((prev) => {
        const updated = [...prev];
        if (updated[updated.length - 1]?.isStreaming) {
          updated.pop();
        }
        return updated;
      });

      // If session error, clear session and try to create new one
      if (errorMessage.includes('Session not found') || errorMessage.includes('expired')) {
        clearStoredSessionId();
        setSessionId(null);
      }
    } finally {
      setIsLoading(false);
    }
  }, [isLoading, currentScope, ensureSession]);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  const clearMessages = useCallback(() => {
    setMessages([]);
  }, []);

  const setScope = useCallback((scope: ScopeContext) => {
    setCurrentScope(scope);
  }, []);

  const setScopeWithSelection = useCallback((selectedText: string) => {
    setCurrentScope({
      type: 'selection',
      selected_text: selectedText,
    });
  }, []);

  return {
    messages,
    isLoading,
    error,
    sessionId,
    currentScope,
    chapterId,
    moduleId,
    sendMessage,
    clearError,
    clearMessages,
    setScope,
    setScopeWithSelection,
  };
}
