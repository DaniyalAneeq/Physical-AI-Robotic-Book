/**
 * OAuth Callback Handler Page
 *
 * Handles OAuth redirects from Google/GitHub.
 * Reads session token from URL fragment and exchanges it for an HttpOnly cookie.
 */

import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

// Determine API base URL
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }
  return 'https://aidd-chatbot-api.onrender.com';
}

export default function OAuthCallback() {
  const [status, setStatus] = useState<'loading' | 'success' | 'error'>('loading');
  const [message, setMessage] = useState('Processing OAuth login...');

  useEffect(() => {
    async function handleOAuthCallback() {
      try {
        // Read token from URL fragment (after #)
        const hash = window.location.hash.substring(1); // Remove leading #
        const params = new URLSearchParams(hash);
        const sessionToken = params.get('session_token');
        const onboardingRequired = params.get('onboarding_required') === 'true';

        if (!sessionToken) {
          setStatus('error');
          setMessage('No session token received from OAuth provider');
          return;
        }

        // Exchange token for cookie by calling backend
        const API_BASE_URL = getApiBaseUrl();
        const response = await fetch(`${API_BASE_URL}/auth/session-from-token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ token: sessionToken }),
          credentials: 'include', // Important: to receive the cookie
        });

        if (!response.ok) {
          const error = await response.json();
          throw new Error(error.detail || 'Failed to establish session');
        }

        const data = await response.json();

        // Verify session was established by checking /auth/session
        const sessionCheck = await fetch(`${API_BASE_URL}/auth/session`, {
          method: 'GET',
          credentials: 'include', // Send the cookie we just received
        });

        if (!sessionCheck.ok) {
          throw new Error('Session verification failed - cookie not set properly');
        }

        const sessionData = await sessionCheck.json();

        // Verify user matches
        if (sessionData.user.id !== data.user.id) {
          throw new Error('Session user mismatch');
        }

        setStatus('success');
        setMessage(`Welcome, ${data.user.name}!`);

        // Clear the URL fragment (remove token from URL for security)
        window.history.replaceState(null, '', window.location.pathname);

        // Redirect after a brief delay (using window.location for proper base path handling)
        setTimeout(() => {
          if (onboardingRequired) {
            window.location.href = '/Physical-AI-Robotic-Book/onboarding';
          } else {
            window.location.href = '/Physical-AI-Robotic-Book/';
          }
        }, 1500);

      } catch (error) {
        console.error('OAuth callback error:', error);
        setStatus('error');
        setMessage(error instanceof Error ? error.message : 'Authentication failed');

        // Redirect to login page after error (using window.location for proper base path handling)
        setTimeout(() => {
          window.location.href = '/Physical-AI-Robotic-Book/login';
        }, 3000);
      }
    }

    handleOAuthCallback();
  }, []);

  return (
    <Layout title="Authenticating...">
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '60vh',
        padding: '2rem',
      }}>
        {status === 'loading' && (
          <>
            <div className="spinner" style={{
              width: '50px',
              height: '50px',
              border: '5px solid #f3f3f3',
              borderTop: '5px solid #3498db',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite',
            }}></div>
            <p style={{ marginTop: '1rem', fontSize: '1.2rem' }}>{message}</p>
          </>
        )}

        {status === 'success' && (
          <>
            <svg
              width="60"
              height="60"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              style={{ color: '#28a745' }}
            >
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path>
              <polyline points="22 4 12 14.01 9 11.01"></polyline>
            </svg>
            <p style={{ marginTop: '1rem', fontSize: '1.2rem', color: '#28a745' }}>
              {message}
            </p>
            <p style={{ marginTop: '0.5rem', color: '#666' }}>
              Redirecting...
            </p>
          </>
        )}

        {status === 'error' && (
          <>
            <svg
              width="60"
              height="60"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              style={{ color: '#dc3545' }}
            >
              <circle cx="12" cy="12" r="10"></circle>
              <line x1="15" y1="9" x2="9" y2="15"></line>
              <line x1="9" y1="9" x2="15" y2="15"></line>
            </svg>
            <p style={{ marginTop: '1rem', fontSize: '1.2rem', color: '#dc3545' }}>
              {message}
            </p>
            <p style={{ marginTop: '0.5rem', color: '#666' }}>
              Redirecting to login...
            </p>
          </>
        )}

        <style>{`
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
        `}</style>
      </div>
    </Layout>
  );
}
