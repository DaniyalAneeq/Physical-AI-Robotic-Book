/**
 * Authentication Hook
 *
 * Provides authentication state and methods throughout the application.
 * Manages user session, login, logout, and registration.
 */

import { useState, useEffect, useCallback, createContext, useContext, ReactNode } from 'react';
import type { User, AuthResponse } from '../services/authApi';
import {
  login as apiLogin,
  register as apiRegister,
  logout as apiLogout,
  getCurrentSession,
  loginWithGoogle as apiLoginWithGoogle,
  getUserPreferences,
} from '../services/authApi';
import { applyUserLocale } from '../utils/localeUtils';

interface AuthContextType {
  user: User | null;
  loading: boolean;
  error: string | null;
  showOnboardingModal: boolean;
  setShowOnboardingModal: (show: boolean) => void;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, name: string, password: string) => Promise<AuthResponse>;
  logout: () => Promise<void>;
  loginWithGoogle: () => void;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

/**
 * Authentication Provider Component
 *
 * Wrap your app with this provider to enable authentication throughout.
 */
export function AuthProvider({ children }: AuthProviderProps) {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [showOnboardingModal, setShowOnboardingModal] = useState(false);

  /**
   * Load current session on mount and apply user preferences
   */
  const loadSession = useCallback(async () => {
    try {
      setLoading(true);
      const session = await getCurrentSession();
      if (session) {
        setUser(session.user);

        // Check if onboarding is required and trigger modal
        if (session.onboarding_required) {
          setShowOnboardingModal(true);
        }

        // Fetch and apply user preferences (including locale)
        try {
          const preferences = await getUserPreferences();
          if (preferences && preferences.preferred_locale) {
            // Apply user's preferred locale
            applyUserLocale(preferences.preferred_locale);
          }
        } catch (prefErr) {
          console.error('Failed to load user preferences:', prefErr);
          // Don't fail session load if preferences fail
        }
      } else {
        setUser(null);
      }
    } catch (err) {
      console.error('Failed to load session:', err);
      setUser(null);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    loadSession();
  }, [loadSession]);

  /**
   * Log in with email and password
   */
  const login = useCallback(async (email: string, password: string) => {
    try {
      setError(null);
      setLoading(true);
      const response = await apiLogin({ email, password });
      setUser(response.user);
      await loadSession();

      // Check if onboarding is required and trigger modal
      if (response.onboarding_required) {
        setShowOnboardingModal(true);
      }

      // Fetch and apply user preferences after successful login
      try {
        const preferences = await getUserPreferences();
        if (preferences && preferences.preferred_locale) {
          applyUserLocale(preferences.preferred_locale);
        }
      } catch (prefErr) {
        console.error('Failed to load user preferences after login:', prefErr);
        // Don't fail login if preferences fail
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Login failed';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * Register a new user
   */
  const register = useCallback(async (email: string, name: string, password: string) => {
    try {
      setError(null);
      setLoading(true);
      const response = await apiRegister({ email, name, password });
      setUser(response.user);
      await loadSession();

      // Check if onboarding is required and trigger modal
      if (response.onboarding_required) {
        setShowOnboardingModal(true);
      }

      // Return the response so the caller can check onboarding_required
      return response;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Registration failed';
      setError(errorMessage);
      throw err;
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * Log out current user
   */
  const logout = useCallback(async () => {
    try {
      setError(null);
      await apiLogout();
      setUser(null);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Logout failed';
      setError(errorMessage);
      throw err;
    }
  }, []);

  /**
   * Initiate Google OAuth login
   */
  const loginWithGoogle = useCallback(() => {
    apiLoginWithGoogle();
  }, []);

  /**
   * Refresh session (e.g., after page reload)
   */
  const refreshSession = useCallback(async () => {
    await loadSession();
  }, [loadSession]);

  const value: AuthContextType = {
    user,
    loading,
    error,
    showOnboardingModal,
    setShowOnboardingModal,
    login,
    register,
    logout,
    loginWithGoogle,
    refreshSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to access authentication context
 *
 * @example
 * ```tsx
 * const { user, login, logout } = useAuth();
 *
 * if (user) {
 *   return <div>Welcome, {user.name}!</div>;
 * }
 * ```
 */
export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

/**
 * Hook to check if user is authenticated
 */
export function useIsAuthenticated(): boolean {
  const { user } = useAuth();
  return user !== null;
}

/**
 * Hook to require authentication
 * Redirects to login if not authenticated
 *
 * Note: This hook should only be used in page components where router is available
 */
export function useRequireAuth(): User {
  const { user, loading } = useAuth();

  useEffect(() => {
    if (!loading && !user) {
      // Redirect to login page
      // Note: This uses window.location to ensure a full page reload
      // In a page component, you could use useHistory instead
      if (typeof window !== 'undefined') {
        window.location.href = '/Physical-AI-Robotic-Book/login';
      }
    }
  }, [user, loading]);

  if (loading) {
    throw new Promise(() => {}); // Suspense
  }

  if (!user) {
    throw new Error('Authentication required');
  }

  return user;
}
