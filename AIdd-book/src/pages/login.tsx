/**
 * Login Page
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import LoginForm from '../components/Auth/LoginForm';
import RegisterForm from '../components/Auth/RegisterForm';
import { useAuth } from '../hooks/useAuth';

export default function LoginPage() {
  const [showRegister, setShowRegister] = useState(false);
  const { user } = useAuth();
  const history = useHistory();

  // Redirect to home if already logged in
  useEffect(() => {
    if (user) {
      history.push('/Physical-AI-Robotic-Book/');
    }
  }, [user, history]);

  if (user) {
    return (
      <Layout title="Login" description="Log in to your account">
        <div style={{ padding: '4rem 2rem', textAlign: 'center' }}>
          <p>You are already logged in. Redirecting...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Login" description="Log in to your account">
      <div style={{ padding: '4rem 2rem', minHeight: '60vh' }}>
        {showRegister ? (
          <RegisterForm
            onSuccess={() => {
              // After registration, onboarding modal will show automatically
              // via OnboardingHandler in Root.tsx
              history.push('/Physical-AI-Robotic-Book/');
            }}
            onSwitchToLogin={() => setShowRegister(false)}
          />
        ) : (
          <LoginForm
            onSuccess={() => {
              // After login, check if onboarding is needed
              // OnboardingHandler in Root.tsx will handle showing the modal
              history.push('/Physical-AI-Robotic-Book/');
            }}
            onSwitchToRegister={() => setShowRegister(true)}
          />
        )}
      </div>
    </Layout>
  );
}
