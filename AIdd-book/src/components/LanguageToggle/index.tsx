/**
 * Language Toggle Component
 *
 * Provides a dropdown for switching between English and Urdu locales.
 * Only visible to authenticated users.
 * Persists user's language preference via backend API.
 */

import React, { useState, useEffect } from 'react';
import type { JSX } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './styles.module.css';

interface LocaleOption {
  code: string;
  label: string;
  direction: 'ltr' | 'rtl';
}

const LOCALES: LocaleOption[] = [
  { code: 'en', label: 'English', direction: 'ltr' },
  { code: 'ur', label: 'اردو', direction: 'rtl' },
];

export default function LanguageToggle(): JSX.Element | null {
  const { user } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();
  const [currentLocale, setCurrentLocale] = useState<string>('en');
  const [isChanging, setIsChanging] = useState(false);

  // Get API URL from site config
  const apiUrl = (siteConfig.customFields?.chatbot as any)?.apiUrl || 'https://e-book-physical-ai-humanoid-robotics.onrender.com';

  // Detect current locale from URL
  useEffect(() => {
    const pathParts = location.pathname.split('/').filter(Boolean);
    const detectedLocale = pathParts[0];

    if (LOCALES.some((l) => l.code === detectedLocale)) {
      setCurrentLocale(detectedLocale);
    } else {
      setCurrentLocale('en');
    }
  }, [location.pathname]);

  /**
   * Handle locale change
   * 1. Update user preference in backend
   * 2. Navigate to new locale URL
   */
  const handleLocaleChange = async (newLocale: string) => {
    if (newLocale === currentLocale || isChanging) {
      return;
    }

    setIsChanging(true);

    try {
      // Update user preference in backend
      if (user) {
        const response = await fetch(
          `${apiUrl}/api/user/preferences`,
          {
            method: 'PUT',
            headers: {
              'Content-Type': 'application/json',
            },
            credentials: 'include', // Include session cookie
            body: JSON.stringify({
              preferred_locale: newLocale,
            }),
          }
        );

        if (!response.ok) {
          throw new Error('Failed to update locale preference');
        }
      }

      // Navigate to new locale
      const currentPath = location.pathname;
      let newPath: string;

      // Handle locale switching in URL
      const pathParts = currentPath.split('/').filter(Boolean);
      const firstPart = pathParts[0];

      if (LOCALES.some((l) => l.code === firstPart)) {
        // Current path has locale prefix, replace it
        pathParts[0] = newLocale;
        newPath = '/' + pathParts.join('/');
      } else {
        // Current path has no locale prefix (default English)
        if (newLocale === 'en') {
          newPath = currentPath; // Stay on English (no prefix)
        } else {
          newPath = `/${newLocale}${currentPath}`;
        }
      }

      // Preserve query params and hash
      newPath += location.search + location.hash;

      // Navigate with page reload (required for Docusaurus locale switch)
      window.location.href = newPath;
    } catch (error) {
      console.error('Failed to change locale:', error);
      setIsChanging(false);
    }
  };

  // Hide toggle if user not authenticated
  if (!user) {
    return null;
  }

  return (
    <div className={styles.languageToggle}>
      <select
        className={styles.languageSelect}
        value={currentLocale}
        onChange={(e) => handleLocaleChange(e.target.value)}
        disabled={isChanging}
        aria-label="Select language"
      >
        {LOCALES.map((locale) => (
          <option key={locale.code} value={locale.code}>
            {locale.label}
          </option>
        ))}
      </select>
      {isChanging && <span className={styles.loadingIndicator}>...</span>}
    </div>
  );
}
