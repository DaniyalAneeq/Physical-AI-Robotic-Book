/**
 * Hook for accessing current locale and locale utilities.
 *
 * Provides access to the user's current locale setting and utilities
 * for locale-aware operations.
 */

import { useEffect, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface UseLocaleResult {
  /** Current locale code (e.g., 'en', 'ur') */
  locale: string;
  /** Whether the current locale uses RTL direction */
  isRTL: boolean;
  /** Set locale programmatically */
  setLocale: (locale: string) => void;
}

/**
 * Hook to access and manage the current locale.
 *
 * @returns Locale utilities including current locale, RTL detection, and setter
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const { locale, isRTL } = useLocale();
 *
 *   return (
 *     <div dir={isRTL ? 'rtl' : 'ltr'}>
 *       Current language: {locale}
 *     </div>
 *   );
 * }
 * ```
 */
export default function useLocale(): UseLocaleResult {
  const { i18n } = useDocusaurusContext();
  const [locale, setLocaleState] = useState(i18n.currentLocale);

  useEffect(() => {
    // Sync with Docusaurus locale changes
    setLocaleState(i18n.currentLocale);
  }, [i18n.currentLocale]);

  const isRTL = locale === 'ur'; // Urdu uses RTL

  const setLocale = (newLocale: string) => {
    // Navigate to the new locale
    const currentPath = window.location.pathname;
    const pathWithoutLocale = currentPath.replace(/^\/(en|ur)/, '');
    const newPath = `/${newLocale}${pathWithoutLocale}`;

    window.location.href = newPath;
  };

  return {
    locale,
    isRTL,
    setLocale,
  };
}
