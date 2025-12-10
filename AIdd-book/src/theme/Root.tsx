import React, { useState } from 'react';
import type { ReactElement, ReactNode } from 'react';
import ChatbotToggle from '@site/src/components/ChatbotToggle';
import ChatbotPanel from '@site/src/components/ChatbotPanel';
import ChatKitChatbot from '../components/ChatKitChatbot';

interface RootProps {
  children: ReactNode;
}

// This component wraps the entire Docusaurus app
// Making the chatbot available on ALL pages including homepage
export default function Root({ children }: RootProps): ReactElement {
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  const toggleChatbot = () => {
    setIsChatbotOpen((prev) => !prev);
  };

  const closeChatbot = () => {
    setIsChatbotOpen(false);
  };

  return (
    <>
      {children}
      <ChatbotToggle isOpen={isChatbotOpen} onClick={toggleChatbot} />
      <ChatbotPanel isOpen={isChatbotOpen} onClose={closeChatbot} />
      {/* <ChatKitChatbot isOpen={isChatbotOpen} onClose={closeChatbot} /> */}
    </>
  );
}
