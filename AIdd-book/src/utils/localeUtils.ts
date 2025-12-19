/**
 * Locale Utilities
 *
 * Utilities for managing user locale preferences and Docusaurus locale switching.
 */

import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

/**
 * Apply user's preferred locale by redirecting to the correct locale path.
 *
 * This function checks if the user's preferred locale differs from the current
 * page locale, and if so, redirects to the same page in the preferred locale.
 *
 * @param preferredLocale - User's preferred locale ('en' or 'ur')
 *
 * @example
 * ```ts
 * // After login, apply user's saved locale
 * applyUserLocale('ur'); // Redirects to /ur/... if currently on /en/...
 * ```
 */
export function applyUserLocale(preferredLocale: string): void {
  // Only run on client-side
  if (!ExecutionEnvironment.canUseDOM) {
    return;
  }

  // Validate locale
  if (preferredLocale !== 'en' && preferredLocale !== 'ur') {
    console.warn(`Invalid locale '${preferredLocale}', skipping auto-apply`);
    return;
  }

  const currentPath = window.location.pathname;

  // Extract current locale from path (e.g., /en/docs/... or /ur/docs/...)
  const localeMatch = currentPath.match(/^\/(en|ur)(\/.*)?$/);
  const currentLocale = localeMatch ? localeMatch[1] : 'en'; // Default to 'en'
  const pathWithoutLocale = localeMatch ? (localeMatch[2] || '/') : currentPath;

  // If preferred locale is different from current, redirect
  if (preferredLocale !== currentLocale) {
    const newPath = `/${preferredLocale}${pathWithoutLocale}`;
    console.log(`Applying user locale: ${currentLocale} → ${preferredLocale}`);
    window.location.href = newPath;
  }
}

/**
 * Get current locale from URL path.
 *
 * @returns Current locale code ('en' or 'ur')
 *
 * @example
 * ```ts
 * // On page /ur/docs/intro
 * const locale = getCurrentLocale(); // Returns 'ur'
 * ```
 */
export function getCurrentLocale(): string {
  if (!ExecutionEnvironment.canUseDOM) {
    return 'en'; // SSR default
  }

  const currentPath = window.location.pathname;
  const localeMatch = currentPath.match(/^\/(en|ur)/);
  return localeMatch ? localeMatch[1] : 'en';
}

/**
 * Switch to a different locale by redirecting.
 *
 * @param targetLocale - Locale to switch to ('en' or 'ur')
 *
 * @example
 * ```ts
 * // Switch to Urdu
 * switchLocale('ur');
 * ```
 */
export function switchLocale(targetLocale: string): void {
  if (!ExecutionEnvironment.canUseDOM) {
    return;
  }

  // Validate locale
  if (targetLocale !== 'en' && targetLocale !== 'ur') {
    console.warn(`Invalid locale '${targetLocale}'`);
    return;
  }

  const currentPath = window.location.pathname;
  const localeMatch = currentPath.match(/^\/(en|ur)(\/.*)?$/);
  const pathWithoutLocale = localeMatch ? (localeMatch[2] || '/') : currentPath;

  const newPath = `/${targetLocale}${pathWithoutLocale}`;
  window.location.href = newPath;
}

/**
 * Check if current page is using RTL locale (Urdu).
 *
 * @returns true if current locale is RTL (Urdu)
 */
export function isRTLLocale(): boolean {
  return getCurrentLocale() === 'ur';
}
