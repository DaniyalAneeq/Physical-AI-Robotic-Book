/**
 * User Menu Component
 *
 * Displays user information and logout button.
 * Shown in the navbar when user is authenticated.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './styles.module.css';

export default function UserMenu() {
  const { user, logout } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => {
        document.removeEventListener('mousedown', handleClickOutside);
      };
    }
  }, [isOpen]);

  if (!user) {
    return null;
  }

  const handleLogout = async () => {
    try {
      await logout();
      setIsOpen(false);
      // Redirect to home page after logout
      window.location.href = '/Physical-AI-Robotic-Book/';
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  // Get user initials for avatar
  const initials = user.name
    .split(' ')
    .map((n) => n[0])
    .join('')
    .toUpperCase()
    .substring(0, 2);

  return (
    <div className={styles.userMenu} ref={menuRef}>
      <button
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="User menu"
        aria-expanded={isOpen}
      >
        <div className={styles.userAvatar}>{initials}</div>
        <span className={styles.userName}>{user.name}</span>
        <svg
          className={styles.dropdownIcon}
          width="12"
          height="12"
          viewBox="0 0 12 12"
          fill="currentColor"
        >
          <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="2" fill="none" />
        </svg>
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownHeader}>
            <p className={styles.dropdownName}>{user.name}</p>
            <p className={styles.dropdownEmail}>{user.email}</p>
          </div>

          <div className={styles.dropdownDivider} />

          <button
            className={styles.dropdownItem}
            onClick={() => {
              setIsOpen(false);
              window.location.href = '/Physical-AI-Robotic-Book/profile';
            }}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 8a3 3 0 100-6 3 3 0 000 6zM8 9c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
            </svg>
            Profile
          </button>

          <button
            className={styles.dropdownItem}
            onClick={() => {
              setIsOpen(false);
              window.location.href = '/Physical-AI-Robotic-Book/settings';
            }}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 10a2 2 0 100-4 2 2 0 000 4zM14 8a1.5 1.5 0 01-1.5 1.5h-1.09l-.59 1.02.54.54a1.5 1.5 0 01-2.12 2.12l-.54-.54-1.02.59V14a1.5 1.5 0 01-3 0v-1.09l-1.02-.59-.54.54a1.5 1.5 0 01-2.12-2.12l.54-.54-.59-1.02H1.5a1.5 1.5 0 010-3h1.09l.59-1.02-.54-.54a1.5 1.5 0 012.12-2.12l.54.54 1.02-.59V1.5a1.5 1.5 0 013 0v1.09l1.02.59.54-.54a1.5 1.5 0 012.12 2.12l-.54.54.59 1.02H14a1.5 1.5 0 011.5 1.5z" />
            </svg>
            Settings
          </button>

          <div className={styles.dropdownDivider} />

          <button
            className={`${styles.dropdownItem} ${styles.logoutItem}`}
            onClick={handleLogout}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M10 14v-2h4V4h-4V2h6v12h-6zM6 12l-1.41-1.41L7.17 8H0V6h7.17L4.59 3.41 6 2l6 6-6 6z" />
            </svg>
            Log Out
          </button>
        </div>
      )}
    </div>
  );
}
