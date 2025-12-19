/**
 * LocalizedModuleContent Component
 *
 * A reusable component for displaying book module content in the user's
 * preferred language with automatic fallback handling.
 */

import React, { useEffect, useState } from 'react';
import useLocale from '@site/src/hooks/useLocale';
import { fetchModuleContent, LocalizedContentResponse } from '@site/src/services/localeApi';
import FallbackBanner from '@site/src/components/FallbackBanner';
import styles from './styles.module.css';

export interface LocalizedModuleContentProps {
  /** Module identifier (e.g., "module-1-chapter-1") */
  moduleId: string;
  /** Custom loading component */
  loadingComponent?: React.ReactNode;
  /** Custom error component */
  errorComponent?: (error: Error) => React.ReactNode;
}

/**
 * Component that fetches and displays module content in the user's locale.
 *
 * Automatically handles:
 * - Fetching content in user's preferred language
 * - Displaying fallback banner for incomplete translations
 * - Loading and error states
 * - RTL/LTR rendering based on locale
 *
 * @param props - Component props
 * @returns Module content with locale support
 *
 * @example
 * ```tsx
 * // In a module page (e.g., docs/module-1/chapter-1.mdx)
 * import LocalizedModuleContent from '@site/src/components/LocalizedModuleContent';
 *
 * <LocalizedModuleContent moduleId="module-1-chapter-1" />
 * ```
 */
export default function LocalizedModuleContent({
  moduleId,
  loadingComponent,
  errorComponent,
}: LocalizedModuleContentProps): JSX.Element {
  const { locale, isRTL } = useLocale();
  const [content, setContent] = useState<LocalizedContentResponse | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    let isMounted = true;

    async function loadContent() {
      setLoading(true);
      setError(null);

      try {
        const data = await fetchModuleContent(moduleId, locale);
        if (isMounted) {
          setContent(data);
          setLoading(false);
        }
      } catch (err) {
        if (isMounted) {
          setError(err instanceof Error ? err : new Error('Failed to load content'));
          setLoading(false);
        }
      }
    }

    loadContent();

    return () => {
      isMounted = false;
    };
  }, [moduleId, locale]);

  // Loading state
  if (loading) {
    if (loadingComponent) {
      return <>{loadingComponent}</>;
    }
    return (
      <div className={styles.loading} role="status" aria-live="polite">
        <div className={styles.spinner} />
        <p>{locale === 'ur' ? 'لوڈ ہو رہا ہے...' : 'Loading...'}</p>
      </div>
    );
  }

  // Error state
  if (error) {
    if (errorComponent) {
      return <>{errorComponent(error)}</>;
    }
    return (
      <div className={styles.error} role="alert">
        <h2>{locale === 'ur' ? 'خرابی' : 'Error'}</h2>
        <p>{error.message}</p>
      </div>
    );
  }

  // Content loaded
  if (!content) {
    return (
      <div className={styles.error} role="alert">
        <p>{locale === 'ur' ? 'مواد دستیاب نہیں' : 'Content not available'}</p>
      </div>
    );
  }

  const showFallback =
    locale === 'ur' &&
    content.translation_status !== 'complete' &&
    content.fallback_message;

  return (
    <div className={styles.moduleContent} dir={isRTL ? 'rtl' : 'ltr'}>
      {/* Fallback banner for incomplete translations */}
      {showFallback && (
        <FallbackBanner
          message={content.fallback_message}
          visible={true}
        />
      )}

      {/* Module title */}
      <h1 className={styles.title}>{content.title}</h1>

      {/* Module content */}
      <div
        className={styles.content}
        dangerouslySetInnerHTML={{ __html: content.content }}
      />

      {/* Translation status indicator (for development/debugging) */}
      {process.env.NODE_ENV === 'development' && (
        <div className={styles.devInfo}>
          <small>
            Locale: {content.locale} | Status: {content.translation_status}
          </small>
        </div>
      )}
    </div>
  );
}
