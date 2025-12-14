import React, { useState } from 'react';
import TextbookChat from '../TextbookChat';
import styles from './styles.module.css';

export default function FloatingChatButton() {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {/* Floating button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
        onClick={() => setIsOpen(true)}
        aria-label="Open chat assistant"
      >
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
        <span className={styles.buttonText}>Ask Assistant</span>
      </button>

      {/* Chat modal */}
      {isOpen && (
        <div className={styles.chatModal}>
          <div className={styles.chatModalHeader}>
            <h3>Textbook Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          <div className={styles.chatModalBody}>
            <TextbookChat compact />
          </div>
        </div>
      )}

      {/* Backdrop */}
      {isOpen && (
        <div
          className={styles.backdrop}
          onClick={() => setIsOpen(false)}
        />
      )}
    </>
  );
}
