import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import TextbookChat from '../TextbookChat';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './styles.module.css';

export default function FloatingChatButton() {
  const [isOpen, setIsOpen] = useState(false);
  const { user, loading } = useAuth();

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
            {!loading && !user ? (
              <div className={styles.authRequired}>
                <svg
                  width="48"
                  height="48"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className={styles.lockIcon}
                >
                  <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
                  <path d="M7 11V7a5 5 0 0 1 10 0v4" />
                </svg>
                <h4>Authentication Required</h4>
                <p>Please log in to access the Textbook Assistant chatbot and get instant answers about Physical AI & Humanoid Robotics.</p>
                <div className={styles.authButtons}>
                  <Link
                    to="/login"
                    className="button button--primary button--md"
                    onClick={() => setIsOpen(false)}
                  >
                    Log In
                  </Link>
                  <Link
                    to="/register"
                    className="button button--secondary button--md"
                    onClick={() => setIsOpen(false)}
                  >
                    Create Account
                  </Link>
                </div>
              </div>
            ) : (
              <TextbookChat compact />
            )}
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
