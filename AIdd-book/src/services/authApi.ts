/**
 * Authentication API Service
 *
 * Handles all authentication-related API calls to the backend.
 * Uses fetch with credentials to handle HttpOnly cookies.
 */

// Determine API base URL based on environment
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }
  // Production: Use deployed backend URL
  // For GitHub Pages deployment, the backend needs to be deployed separately
  return 'https://aidd-chatbot-api.onrender.com';
}

const API_BASE_URL = getApiBaseUrl();

export interface User {
  id: string;
  email: string;
  name: string;
  email_verified_at: string | null;
  onboarding_completed: boolean;
  preferred_locale?: string; // User's preferred language ('en' or 'ur')
  created_at: string;
  updated_at: string;
}

export interface Session {
  id: string;
  user_id: string;
  last_used_at: string;
  expires_at: string;
  ip_address: string | null;
  user_agent: string | null;
  created_at: string;
}

export interface AuthResponse {
  user: User;
  session: Session;
  message: string;
  onboarding_required: boolean;
}

export interface RegisterData {
  email: string;
  name: string;
  password: string;
}

export interface LoginData {
  email: string;
  password: string;
}

export interface AuthError {
  detail: string;
}

export interface OnboardingData {
  user_type: string;
  area_of_interest: string;
  experience_level: string;
  topics_of_interest?: string[];
}

export interface OnboardingOptions {
  user_types: string[];
  areas_of_interest: string[];
  experience_levels: string[];
  topics_of_interest: string[];
}

export interface OnboardingProfile {
  id: string;
  user_id: string;
  user_type: string;
  area_of_interest: string;
  experience_level: string;
  topics_of_interest: string[] | null;
  created_at: string;
  updated_at: string;
}

/**
 * Register a new user
 */
export async function register(data: RegisterData): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/auth/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
    credentials: 'include', // Important: Include cookies
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Registration failed');
  }

  return response.json();
}

/**
 * Log in an existing user
 */
export async function login(data: LoginData): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/auth/login`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
    credentials: 'include',
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Login failed');
  }

  return response.json();
}

/**
 * Log out the current user
 */
export async function logout(): Promise<void> {
  const response = await fetch(`${API_BASE_URL}/auth/logout`, {
    method: 'POST',
    credentials: 'include',
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Logout failed');
  }
}

/**
 * Get current session and user information
 */
export async function getCurrentSession(): Promise<AuthResponse | null> {
  try {
    const response = await fetch(`${API_BASE_URL}/auth/session`, {
      credentials: 'include',
    });

    if (response.status === 401) {
      // Not authenticated
      return null;
    }

    if (!response.ok) {
      throw new Error('Failed to get session');
    }

    return response.json();
  } catch (error) {
    console.error('Error getting session:', error);
    return null;
  }
}

/**
 * Initiate Google OAuth login
 */
export function loginWithGoogle(): void {
  window.location.href = `${API_BASE_URL}/auth/oauth/google`;
}

/**
 * Check if user is authenticated (has valid session)
 */
export async function isAuthenticated(): Promise<boolean> {
  const session = await getCurrentSession();
  return session !== null;
}

/**
 * Get onboarding options
 */
export async function getOnboardingOptions(): Promise<OnboardingOptions> {
  const response = await fetch(`${API_BASE_URL}/auth/onboarding/options`, {
    credentials: 'include',
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Failed to get onboarding options');
  }

  return response.json();
}

/**
 * Complete onboarding
 */
export async function completeOnboarding(data: OnboardingData): Promise<OnboardingProfile> {
  const response = await fetch(`${API_BASE_URL}/auth/onboarding/complete`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
    credentials: 'include',
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Failed to complete onboarding');
  }

  return response.json();
}

/**
 * Get current user's onboarding profile
 */
export async function getOnboardingProfile(): Promise<OnboardingProfile | null> {
  try {
    const response = await fetch(`${API_BASE_URL}/auth/onboarding/profile`, {
      credentials: 'include',
    });

    if (response.status === 404 || response.status === 401) {
      return null;
    }

    if (!response.ok) {
      throw new Error('Failed to get onboarding profile');
    }

    return response.json();
  } catch (error) {
    console.error('Error getting onboarding profile:', error);
    return null;
  }
}

/**
 * User preferences interface
 */
export interface UserPreferences {
  preferred_locale: string; // 'en' or 'ur'
}

/**
 * Get user preferences (including preferred locale)
 */
export async function getUserPreferences(): Promise<UserPreferences | null> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/user/preferences`, {
      credentials: 'include',
    });

    if (response.status === 401) {
      // Not authenticated
      return null;
    }

    if (!response.ok) {
      throw new Error('Failed to get user preferences');
    }

    return response.json();
  } catch (error) {
    console.error('Error getting user preferences:', error);
    return null;
  }
}

/**
 * Update user preferences
 */
export async function updateUserPreferences(
  preferences: Partial<UserPreferences>
): Promise<UserPreferences> {
  const response = await fetch(`${API_BASE_URL}/api/user/preferences`, {
    method: 'PUT',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(preferences),
    credentials: 'include',
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Failed to update user preferences');
  }

  return response.json();
}
