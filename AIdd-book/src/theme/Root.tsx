import React, { useEffect, useState } from 'react';
import type { ReactElement, ReactNode } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import FloatingChatButton from '@site/src/components/FloatingChatButton';
import { AuthProvider, useAuth } from '../hooks/useAuth';
import OnboardingModal from '../components/Auth/OnboardingModal';

interface RootProps {
  children: ReactNode;
}

// Onboarding handler component
function OnboardingHandler(): ReactElement | null {
  const { user, refreshSession } = useAuth();
  const [showOnboarding, setShowOnboarding] = useState(false);
  const location = useLocation();
  const history = useHistory();

  useEffect(() => {
    // Only show onboarding if explicitly requested via URL parameter (after registration/OAuth)
    const params = new URLSearchParams(location.search);
    const onboardingParam = params.get('onboarding');

    if (onboardingParam === 'required' && user && !user.onboarding_completed) {
      setShowOnboarding(true);
      // Clean up URL parameter
      params.delete('onboarding');
      const newSearch = params.toString();
      const newUrl = location.pathname + (newSearch ? `?${newSearch}` : '');
      history.replace(newUrl);
    } else if (user && user.onboarding_completed && showOnboarding) {
      // User has completed onboarding, hide the modal
      setShowOnboarding(false);
    }
  }, [user, location.search, history, location.pathname, showOnboarding]);

  const handleOnboardingComplete = async () => {
    setShowOnboarding(false);
    // Refresh session to update user.onboarding_completed
    await refreshSession();
    // Redirect to the main book page if not already there
    if (!location.pathname.includes('/Physical-AI-Robotic-Book')) {
      history.push('/Physical-AI-Robotic-Book');
    }
  };

  return (
    <OnboardingModal
      isOpen={showOnboarding}
      onComplete={handleOnboardingComplete}
    />
  );
}

// This component wraps the entire Docusaurus app
// Making the chatbot and authentication available on ALL pages
export default function Root({ children }: RootProps): ReactElement {
  return (
    <AuthProvider>
      {children}
      <FloatingChatButton />
      <OnboardingHandler />
    </AuthProvider>
  );
}
