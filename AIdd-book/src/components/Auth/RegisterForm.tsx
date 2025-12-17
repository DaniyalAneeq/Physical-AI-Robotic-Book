/**
 * Registration Form Component
 *
 * Allows new users to create an account.
 */

import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './styles.module.css';

interface RegisterFormProps {
  onSuccess?: () => void;
  onSwitchToLogin?: () => void;
}

export default function RegisterForm({ onSuccess, onSwitchToLogin }: RegisterFormProps) {
  const { register, loginWithGoogle, error } = useAuth();
  const [email, setEmail] = useState('');
  const [name, setName] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [localError, setLocalError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLocalError(null);

    // Validation
    if (password.length < 8) {
      setLocalError('Password must be at least 8 characters long');
      return;
    }

    if (password !== confirmPassword) {
      setLocalError('Passwords do not match');
      return;
    }

    setIsLoading(true);

    try {
      const response = await register(email, name, password);

      // If onboarding is required, redirect with parameter
      if (response.onboarding_required) {
        window.location.href = '/Physical-AI-Robotic-Book/?onboarding=required';
      } else {
        onSuccess?.();
      }
    } catch (err) {
      setLocalError(err instanceof Error ? err.message : 'Registration failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleGoogleSignup = () => {
    loginWithGoogle();
  };

  return (
    <div className={styles.authForm}>
      <h2>Create Account</h2>
      <p className={styles.subtitle}>Start your learning journey today.</p>

      {(localError || error) && (
        <div className={styles.error}>{localError || error}</div>
      )}

      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label htmlFor="name">Full Name</label>
          <input
            id="name"
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="John Doe"
            required
            disabled={isLoading}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="your.email@example.com"
            required
            disabled={isLoading}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="At least 8 characters"
            required
            minLength={8}
            disabled={isLoading}
          />
          <small>Must be at least 8 characters long</small>
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="confirmPassword">Confirm Password</label>
          <input
            id="confirmPassword"
            type="password"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            placeholder="Re-enter your password"
            required
            disabled={isLoading}
          />
        </div>

        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Creating account...' : 'Create Account'}
        </button>
      </form>

      <div className={styles.divider}>
        <span>OR</span>
      </div>

      <button
        type="button"
        className={styles.googleButton}
        onClick={handleGoogleSignup}
        disabled={isLoading}
      >
        <svg className={styles.googleIcon} viewBox="0 0 24 24">
          <path
            fill="#4285F4"
            d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
          />
          <path
            fill="#34A853"
            d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
          />
          <path
            fill="#FBBC05"
            d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
          />
          <path
            fill="#EA4335"
            d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
          />
        </svg>
        Sign up with Google
      </button>

      {onSwitchToLogin && (
        <p className={styles.switchAuth}>
          Already have an account?{' '}
          <button
            type="button"
            onClick={onSwitchToLogin}
            className={styles.linkButton}
          >
            Log in
          </button>
        </p>
      )}
    </div>
  );
}
