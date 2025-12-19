/**
 * FallbackBanner Component
 *
 * Displays an informational banner when content is not available in the
 * user's preferred language and is being shown in English as a fallback.
 */

import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface FallbackBannerProps {
  /** Custom message to display (optional) */
  message?: string;
  /** Additional CSS classes */
  className?: string;
  /** Whether to show the banner (default: true) */
  visible?: boolean;
}

/**
 * Banner component indicating content fallback to English.
 *
 * @param props - Component props
 * @returns Banner element or null if not visible
 *
 * @example
 * ```tsx
 * <FallbackBanner
 *   message="This chapter is not yet available in Urdu"
 *   visible={translationStatus !== 'complete'}
 * />
 * ```
 */
export default function FallbackBanner({
  message = 'This content is not yet available in Urdu. Showing English version. Translation coming soon!',
  className,
  visible = true,
}: FallbackBannerProps): JSX.Element | null {
  if (!visible) {
    return null;
  }

  return (
    <div className={clsx(styles.fallbackBanner, className)} role="alert">
      <div className={styles.iconContainer}>
        <svg
          className={styles.icon}
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 20 20"
          fill="currentColor"
          aria-hidden="true"
        >
          <path
            fillRule="evenodd"
            d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z"
            clipRule="evenodd"
          />
        </svg>
      </div>
      <div className={styles.content}>
        <p className={styles.message}>{message}</p>
      </div>
    </div>
  );
}
