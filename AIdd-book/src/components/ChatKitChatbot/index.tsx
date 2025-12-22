import React, { useEffect, useState } from 'react';
import type { ReactElement } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import styles from './styles.module.css';

// API configuration - determine base URL based on environment
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
  }
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
  }
  // Production: Use deployed backend URL
  return process.env.REACT_APP_API_URL || 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
}

interface ChatKitChatbotProps {
  isOpen: boolean;
  onClose: () => void;
}

// Inner component that uses the hook - only rendered when isOpen is true
function ChatKitContent({ onClose }: { onClose: () => void }): ReactElement {
  const [isClient, setIsClient] = useState(false);
  const apiBaseUrl = getApiBaseUrl();

  // Ensure client-side only rendering for ChatKit
  useEffect(() => {
    setIsClient(true);
  }, []);

  const { control } = useChatKit({
    api: {
      url: `${apiBaseUrl}/api/chatkit`,
      domainKey: 'aidd-textbook',
    },
    theme: {
      colorScheme: 'light', // Use light theme
      radius: 'round',
      color: {
        accent: { primary: '#2e8555', level: 2 }, // Docusaurus primary color
      },
    },
    header: {
      enabled: false, // We have our own header
    },
    startScreen: {
      greeting: 'How can I help you learn about Physical AI & Humanoid Robotics?',
      prompts: [
        {
          label: 'What is ROS 2?',
          prompt: 'What is ROS 2 and how does it differ from ROS 1?',
        },
        {
          label: 'Digital Twins',
          prompt: 'Explain what a digital twin is in robotics',
        },
        {
          label: 'Getting Started',
          prompt: 'How do I get started with humanoid robotics development?',
        },
      ],
    },
    composer: {
      placeholder: 'Ask about the textbook...',
    },
    threadItemActions: {
      feedback: true,
      retry: true,
    },
    onError: (event) => {
      console.error('ChatKit error:', event.error);
    },
  });

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.chatbotHeader}>
        <div className={styles.headerContent}>
          <span className={styles.headerIcon}>📚</span>
          <div className={styles.headerText}>
            <h3>Textbook Assistant</h3>
            <span>AI-powered help for your learning</span>
          </div>
        </div>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chatbot"
          type="button"
        >
          ✕
        </button>
      </div>
      <div className={styles.chatbotBody}>
        {isClient ? (
          <ChatKit control={control} className={styles.chatkit} />
        ) : (
          <div className={styles.loading}>Loading chatbot...</div>
        )}
      </div>
    </div>
  );
}

// Main component - conditionally renders ChatKitContent
export default function ChatKitChatbot({ isOpen, onClose }: ChatKitChatbotProps): ReactElement | null {
  // Don't render anything if not open - this prevents the useChatKit hook from being called
  if (!isOpen) {
    return null;
  }

  return <ChatKitContent onClose={onClose} />;
}
