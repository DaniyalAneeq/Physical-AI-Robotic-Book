/**
 * API service for fetching localized content.
 *
 * Provides functions to retrieve book module content in the user's
 * preferred language from the backend API.
 */

import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export interface LocalizedContentResponse {
  module_id: string;
  locale: string;
  content: string;
  title: string;
  translation_status: 'complete' | 'partial' | 'unavailable';
  fallback_message?: string;
}

/**
 * Fetch module content in the specified locale.
 *
 * @param moduleId - The module identifier (e.g., "module-1-chapter-1")
 * @param locale - The desired locale code ('en' or 'ur')
 * @returns Promise resolving to localized content
 * @throws Error if the request fails or module is not found
 *
 * @example
 * ```ts
 * const content = await fetchModuleContent('module-1-chapter-1', 'ur');
 * console.log(content.title); // Module title in Urdu
 * ```
 */
export async function fetchModuleContent(
  moduleId: string,
  locale: string
): Promise<LocalizedContentResponse> {
  // Only run on client-side
  if (!ExecutionEnvironment.canUseDOM) {
    throw new Error('fetchModuleContent can only be called on the client side');
  }

  const url = `${API_BASE_URL}/api/content/${encodeURIComponent(moduleId)}`;

  try {
    const response = await fetch(url, {
      method: 'GET',
      headers: {
        'Accept': 'application/json',
        'Accept-Language': locale,
        'Content-Type': 'application/json',
        // Include credentials for authentication
        'Authorization': `Bearer ${getAuthToken()}`,
      },
      credentials: 'include',
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error(`Module '${moduleId}' not found`);
      }
      if (response.status === 401) {
        throw new Error('Authentication required. Please log in.');
      }
      throw new Error(`Failed to fetch content: ${response.statusText}`);
    }

    const data: LocalizedContentResponse = await response.json();
    return data;
  } catch (error) {
    if (error instanceof Error) {
      throw error;
    }
    throw new Error('An unexpected error occurred while fetching content');
  }
}

/**
 * Get authentication token from localStorage or cookies.
 * This is a placeholder - actual implementation depends on auth setup.
 *
 * @returns JWT token or empty string if not authenticated
 */
function getAuthToken(): string {
  if (!ExecutionEnvironment.canUseDOM) {
    return '';
  }

  // Try localStorage first (common pattern)
  const token = localStorage.getItem('auth_token');
  if (token) {
    return token;
  }

  // Try to extract from cookie as fallback
  const cookies = document.cookie.split(';');
  for (const cookie of cookies) {
    const [name, value] = cookie.trim().split('=');
    if (name === 'auth_token') {
      return value;
    }
  }

  return '';
}

/**
 * Prefetch module content for better performance.
 *
 * @param moduleId - The module identifier to prefetch
 * @param locale - The locale to prefetch
 *
 * @example
 * ```ts
 * // Prefetch next chapter when user is reading current chapter
 * prefetchModuleContent('module-1-chapter-2', 'ur');
 * ```
 */
export function prefetchModuleContent(
  moduleId: string,
  locale: string
): void {
  if (!ExecutionEnvironment.canUseDOM) {
    return;
  }

  // Trigger fetch in background without awaiting
  fetchModuleContent(moduleId, locale).catch(() => {
    // Silent fail for prefetch
  });
}
