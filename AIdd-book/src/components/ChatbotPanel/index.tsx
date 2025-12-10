import React, { useEffect, useRef, useState, useCallback } from 'react';
import type { ReactElement } from 'react';
import styles from './styles.module.css';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import ScopeSelector, { type ScopeType } from './ScopeSelector';
import { useChatbot } from '../../hooks/useChatbot';
import { useTextSelection } from '../../hooks/useTextSelection';

interface ChatbotPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

const SUGGESTED_QUESTIONS = [
  "What is ROS 2 and how does it differ from ROS 1?",
  "Explain what a digital twin is in robotics",
  "How do I create a ROS 2 publisher node?",
];

export default function ChatbotPanel({ isOpen, onClose }: ChatbotPanelProps): ReactElement | null {
  const {
    messages,
    isLoading,
    error,
    currentScope,
    chapterId,
    moduleId,
    sendMessage,
    clearError,
    setScope,
    setScopeWithSelection,
  } = useChatbot();

  const { selectedText, hasSelection } = useTextSelection();
  const [announcement, setAnnouncement] = useState('');
  const [lastQuery, setLastQuery] = useState('');

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const panelRef = useRef<HTMLDivElement>(null);
  const closeButtonRef = useRef<HTMLButtonElement>(null);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Announce new messages to screen readers
  useEffect(() => {
    if (messages.length > 0) {
      const lastMessage = messages[messages.length - 1];
      if (lastMessage.role === 'assistant' && !lastMessage.isStreaming) {
        setAnnouncement(`Assistant response: ${lastMessage.content.slice(0, 100)}...`);
      }
    }
  }, [messages]);

  // Focus management - focus input when panel opens
  useEffect(() => {
    if (isOpen && panelRef.current) {
      const input = panelRef.current.querySelector('textarea');
      input?.focus();
    }
  }, [isOpen]);

  // Handle keyboard navigation
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      // Escape to close
      if (e.key === 'Escape') {
        e.preventDefault();
        onClose();
        return;
      }

      // Focus trap - Tab within the panel
      if (e.key === 'Tab' && panelRef.current) {
        const focusableElements = panelRef.current.querySelectorAll<HTMLElement>(
          'button, textarea, select, [tabindex]:not([tabindex="-1"])'
        );
        const firstElement = focusableElements[0];
        const lastElement = focusableElements[focusableElements.length - 1];

        if (e.shiftKey && document.activeElement === firstElement) {
          e.preventDefault();
          lastElement?.focus();
        } else if (!e.shiftKey && document.activeElement === lastElement) {
          e.preventDefault();
          firstElement?.focus();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  // Handle scope change from selector
  const handleScopeChange = useCallback((scopeType: ScopeType) => {
    switch (scopeType) {
      case 'page':
        if (chapterId) {
          setScope({ type: 'page', chapter_id: chapterId });
          setAnnouncement('Scope changed to current page');
        }
        break;
      case 'selection':
        if (selectedText) {
          setScopeWithSelection(selectedText);
          setAnnouncement('Scope changed to selected text');
        }
        break;
      case 'module':
        if (moduleId) {
          setScope({ type: 'module', module_id: moduleId });
          setAnnouncement('Scope changed to current module');
        }
        break;
      case 'all':
      default:
        setScope({ type: 'all' });
        setAnnouncement('Scope changed to all content');
        break;
    }
  }, [chapterId, moduleId, selectedText, setScope, setScopeWithSelection]);

  // Handle message send
  const handleSendMessage = useCallback((message: string) => {
    setLastQuery(message);
    sendMessage(message);
  }, [sendMessage]);

  // Handle retry
  const handleRetry = useCallback(() => {
    if (lastQuery) {
      clearError();
      sendMessage(lastQuery);
    }
  }, [lastQuery, clearError, sendMessage]);

  // Handle suggested question click
  const handleSuggestedQuestion = useCallback((question: string) => {
    handleSendMessage(question);
  }, [handleSendMessage]);

  if (!isOpen) return null;

  return (
    <div
      ref={panelRef}
      className={styles.panel}
      role="dialog"
      aria-label="Textbook Assistant chatbot"
      aria-modal="true"
      aria-describedby="chatbot-description"
    >
      {/* Visually hidden description for screen readers */}
      <div id="chatbot-description" className={styles.srOnly}>
        Chat assistant for the Physical AI and Humanoid Robotics textbook.
        Ask questions and receive answers with citations from the textbook content.
        Press Escape to close.
      </div>

      {/* Screen reader announcements */}
      <div
        className={styles.srOnly}
        role="status"
        aria-live="polite"
        aria-atomic="true"
      >
        {announcement}
      </div>

      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <div className={styles.headerIcon}>
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              aria-hidden="true"
            >
              <path d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253"></path>
            </svg>
          </div>
          <div>
            <h2 className={styles.title} id="chatbot-title">Textbook Assistant</h2>
            <p className={styles.subtitle}>AI-powered help for your learning</p>
          </div>
        </div>
        <button
          ref={closeButtonRef}
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chatbot"
          type="button"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            aria-hidden="true"
          >
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        </button>
      </div>

      {/* Scope Selector */}
      <ScopeSelector
        currentScope={currentScope.type}
        hasSelection={hasSelection}
        selectedText={selectedText || undefined}
        onScopeChange={handleScopeChange}
        chapterId={chapterId || undefined}
        moduleId={moduleId || undefined}
      />

      {/* Messages Area */}
      <div
        className={styles.messages}
        aria-label="Chat messages"
        role="log"
        aria-live="polite"
        aria-atomic="false"
        tabIndex={0}
      >
        {messages.length === 0 && (
          <div className={styles.welcome}>
            <div className={styles.welcomeIcon}>
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="32"
                height="32"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="1.5"
                aria-hidden="true"
              >
                <circle cx="12" cy="12" r="10"></circle>
                <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path>
                <line x1="12" y1="17" x2="12.01" y2="17"></line>
              </svg>
            </div>
            <h3 className={styles.welcomeTitle}>How can I help you today?</h3>
            <p>Ask me anything about the Physical AI & Humanoid Robotics textbook!</p>
            <p className={styles.hint}>
              I can explain concepts, clarify code examples, and help you understand robotics topics.
            </p>
            {hasSelection && (
              <p className={styles.hint}>
                Tip: You have text selected. Switch to "Selected Text" scope to ask about it specifically.
              </p>
            )}

            {/* Suggested Questions */}
            <div className={styles.suggestedQuestions}>
              {SUGGESTED_QUESTIONS.map((question, index) => (
                <button
                  key={index}
                  className={styles.suggestedQuestion}
                  onClick={() => handleSuggestedQuestion(question)}
                  type="button"
                >
                  {question}
                </button>
              ))}
            </div>
          </div>
        )}

        {messages.map((message, index) => (
          <ChatMessage
            key={index}
            role={message.role}
            content={message.content}
            citations={message.citations}
            isStreaming={message.isStreaming}
          />
        ))}

        {/* Loading Indicator */}
        {isLoading && messages[messages.length - 1]?.role !== 'assistant' && (
          <div className={styles.loading} role="status" aria-label="Loading response">
            <div className={styles.loadingDots}>
              <span className={styles.loadingDot} aria-hidden="true"></span>
              <span className={styles.loadingDot} aria-hidden="true"></span>
              <span className={styles.loadingDot} aria-hidden="true"></span>
            </div>
            <span className={styles.loadingText}>Thinking...</span>
            <span className={styles.srOnly}>Generating response...</span>
          </div>
        )}

        {/* Error State */}
        {error && (
          <div className={styles.error} role="alert" aria-live="assertive">
            <div className={styles.errorIcon}>
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                aria-hidden="true"
              >
                <circle cx="12" cy="12" r="10"></circle>
                <line x1="12" y1="8" x2="12" y2="12"></line>
                <line x1="12" y1="16" x2="12.01" y2="16"></line>
              </svg>
            </div>
            <div className={styles.errorContent}>
              <p className={styles.errorMessage}>{error}</p>
              <div className={styles.errorActions}>
                <button
                  onClick={handleRetry}
                  className={styles.retryButton}
                  type="button"
                >
                  Try Again
                </button>
                <button
                  onClick={clearError}
                  className={styles.dismissError}
                  aria-label="Dismiss error"
                  type="button"
                >
                  Dismiss
                </button>
              </div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <ChatInput
        onSubmit={handleSendMessage}
        disabled={isLoading}
        placeholder={isLoading ? 'Waiting for response...' : 'Ask a question about the textbook...'}
      />
    </div>
  );
}
