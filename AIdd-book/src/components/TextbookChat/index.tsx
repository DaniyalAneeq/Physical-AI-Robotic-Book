import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: string[];
  timestamp: Date;
}

interface TextbookChatProps {
  apiUrl?: string;
  moduleFilter?: string;
  compact?: boolean;
}

export default function TextbookChat({
  apiUrl,
  moduleFilter,
  compact = false,
}: TextbookChatProps): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const chatbotConfig = (siteConfig.customFields?.chatbot as any) || {};
  const finalApiUrl = apiUrl || `${chatbotConfig.apiUrl || 'http://localhost:8000'}/chatkit`;
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isStreaming, setIsStreaming] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const abortControllerRef = useRef<AbortController | null>(null);

  // Initialize session ID from localStorage
  useEffect(() => {
    let storedSessionId = localStorage.getItem('textbook_chat_session_id');
    if (!storedSessionId) {
      storedSessionId = crypto.randomUUID();
      localStorage.setItem('textbook_chat_session_id', storedSessionId);
    }
    setSessionId(storedSessionId);
  }, []);

  // Scroll to bottom on new messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isStreaming || !sessionId) return;

    const userMessage: Message = {
      id: crypto.randomUUID(),
      role: 'user',
      content: input.trim(),
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsStreaming(true);
    setError(null);

    // Prepare assistant message placeholder
    const assistantMessageId = crypto.randomUUID();
    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      citations: [],
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, assistantMessage]);

    // Create abort controller for cancellation
    const abortController = new AbortController();
    abortControllerRef.current = abortController;

    try {
      const response = await fetch(finalApiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.content,
          session_id: sessionId,
          conversation_id: conversationId,
          context: moduleFilter ? { module_filter: moduleFilter } : {},
        }),
        signal: abortController.signal,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('No response body');
      }

      let accumulatedContent = '';
      const citations: string[] = [];

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (!line.trim() || !line.startsWith('data:')) continue;

          const data = line.substring(5).trim();
          if (data === '[DONE]') continue;

          try {
            const parsed = JSON.parse(data);

            if (parsed.type === 'token') {
              accumulatedContent += parsed.content;
              setMessages(prev =>
                prev.map(msg =>
                  msg.id === assistantMessageId
                    ? { ...msg, content: accumulatedContent }
                    : msg
                )
              );
            } else if (parsed.type === 'citation') {
              citations.push(parsed.content);
              setMessages(prev =>
                prev.map(msg =>
                  msg.id === assistantMessageId
                    ? { ...msg, citations }
                    : msg
                )
              );
            } else if (parsed.conversation_id) {
              setConversationId(parsed.conversation_id);
            }
          } catch (parseError) {
            console.error('Error parsing SSE data:', parseError);
          }
        }
      }
    } catch (err: any) {
      if (err.name === 'AbortError') {
        console.log('Request aborted');
      } else {
        console.error('Error streaming response:', err);
        setError('Failed to get response. Please try again.');
      }
    } finally {
      setIsStreaming(false);
      abortControllerRef.current = null;
    }
  };

  const handleNewChat = () => {
    setMessages([]);
    setConversationId(null);
    setError(null);
  };

  return (
    <div className={`${styles.chatContainer} ${compact ? styles.compact : ''}`}>
      <div className={styles.chatHeader}>
        <h3>Textbook Assistant</h3>
        <button onClick={handleNewChat} className={styles.newChatBtn}>
          New Chat
        </button>
      </div>

      {error && (
        <div className={styles.errorBanner}>
          {error}
        </div>
      )}

      <div className={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div className={styles.welcomeScreen}>
            <h4>Ask me anything about Physical AI & Robotics!</h4>
            <div className={styles.suggestedPrompts}>
              <button
                onClick={() => setInput('What is ROS 2?')}
                className={styles.promptBtn}
              >
                What is ROS 2?
              </button>
              <button
                onClick={() => setInput('Explain URDF and how it is used in robotics')}
                className={styles.promptBtn}
              >
                Explain URDF
              </button>
              <button
                onClick={() => setInput('What are digital twins in robotics?')}
                className={styles.promptBtn}
              >
                Digital Twins
              </button>
            </div>
          </div>
        ) : (
          messages.map(msg => (
            <div key={msg.id} className={`${styles.message} ${styles[msg.role]}`}>
              <div className={styles.messageContent}>
                <div className={styles.messageRole}>
                  {msg.role === 'user' ? 'You' : 'Assistant'}
                </div>
                <div className={styles.messageText}>{msg.content}</div>
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Sources:</strong>
                    <ul>
                      {msg.citations.map((citation, idx) => (
                        <li key={idx}>{citation}</li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={e => setInput(e.target.value)}
          placeholder="Ask a question about the textbook..."
          className={styles.inputField}
          disabled={isStreaming || !sessionId}
        />
        <button
          type="submit"
          disabled={!input.trim() || isStreaming || !sessionId}
          className={styles.sendBtn}
        >
          {isStreaming ? 'Thinking...' : 'Send'}
        </button>
      </form>
    </div>
  );
}
