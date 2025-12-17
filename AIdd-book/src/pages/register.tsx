/**
 * Register Page
 */

import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import RegisterForm from '../components/Auth/RegisterForm';
import { useAuth } from '../hooks/useAuth';

export default function RegisterPage() {
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
      <Layout title="Register" description="Create a new account">
        <div style={{ padding: '4rem 2rem', textAlign: 'center' }}>
          <p>You are already logged in. Redirecting...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Register" description="Create a new account">
      <div style={{ padding: '4rem 2rem', minHeight: '60vh' }}>
        <RegisterForm
          onSuccess={() => {
            history.push('/Physical-AI-Robotic-Book/');
          }}
          onSwitchToLogin={() => {
            history.push('/Physical-AI-Robotic-Book/login');
          }}
        />
      </div>
    </Layout>
  );
}
