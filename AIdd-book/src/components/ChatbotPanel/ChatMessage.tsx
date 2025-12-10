import React, { useMemo } from 'react';
import type { ReactElement } from 'react';
import styles from './styles.module.css';

interface Citation {
  chunk_id: string;
  chapter_title: string;
  section_title: string;
  paragraph_number: number;
  relevance_score: number;
  excerpt: string;
}

interface ChatMessageProps {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  isStreaming?: boolean;
}

/**
 * Simple markdown-like formatting for chat messages.
 * Handles bold, italic, inline code, and code blocks.
 */
function formatContent(text: string): React.ReactNode[] {
  const elements: React.ReactNode[] = [];

  // Split by code blocks first
  const codeBlockRegex = /```(\w*)\n?([\s\S]*?)```/g;
  let lastIndex = 0;
  let match: RegExpExecArray | null;
  let keyIndex = 0;

  const processInlineFormatting = (segment: string): React.ReactNode[] => {
    const inlineElements: React.ReactNode[] = [];

    // Process inline code, bold, and italic
    const inlineRegex = /(`[^`]+`)|(\*\*[^*]+\*\*)|(\*[^*]+\*)/g;
    let inlineLastIndex = 0;
    let inlineMatch: RegExpExecArray | null;

    while ((inlineMatch = inlineRegex.exec(segment)) !== null) {
      // Add text before the match
      if (inlineMatch.index > inlineLastIndex) {
        inlineElements.push(segment.slice(inlineLastIndex, inlineMatch.index));
      }

      const matched = inlineMatch[0];
      if (matched.startsWith('`')) {
        // Inline code
        inlineElements.push(
          <code key={`inline-${keyIndex++}`}>
            {matched.slice(1, -1)}
          </code>
        );
      } else if (matched.startsWith('**')) {
        // Bold
        inlineElements.push(
          <strong key={`bold-${keyIndex++}`}>
            {matched.slice(2, -2)}
          </strong>
        );
      } else if (matched.startsWith('*')) {
        // Italic
        inlineElements.push(
          <em key={`em-${keyIndex++}`}>
            {matched.slice(1, -1)}
          </em>
        );
      }

      inlineLastIndex = inlineMatch.index + matched.length;
    }

    // Add remaining text
    if (inlineLastIndex < segment.length) {
      inlineElements.push(segment.slice(inlineLastIndex));
    }

    return inlineElements.length > 0 ? inlineElements : [segment];
  };

  while ((match = codeBlockRegex.exec(text)) !== null) {
    // Add text before the code block
    if (match.index > lastIndex) {
      const beforeText = text.slice(lastIndex, match.index);
      elements.push(
        <span key={`text-${keyIndex++}`}>
          {processInlineFormatting(beforeText)}
        </span>
      );
    }

    // Add the code block
    const language = match[1] || 'text';
    const code = match[2].trim();
    elements.push(
      <pre key={`pre-${keyIndex++}`} data-language={language}>
        <code>{code}</code>
      </pre>
    );

    lastIndex = match.index + match[0].length;
  }

  // Add remaining text after last code block
  if (lastIndex < text.length) {
    const remainingText = text.slice(lastIndex);
    elements.push(
      <span key={`text-${keyIndex++}`}>
        {processInlineFormatting(remainingText)}
      </span>
    );
  }

  return elements.length > 0 ? elements : [text];
}

/**
 * Get score badge class based on relevance score.
 */
function getScoreClass(score: number): string {
  if (score >= 0.7) return styles.high;
  if (score >= 0.5) return styles.medium;
  return styles.low;
}

export default function ChatMessage({
  role,
  content,
  citations,
  isStreaming,
}: ChatMessageProps): ReactElement {
  // Memoize formatted content to avoid re-processing on every render
  const formattedContent = useMemo(() => {
    if (!content) return null;
    return formatContent(content);
  }, [content]);

  // Sort citations by relevance score (highest first) and limit to top 3
  const topCitations = useMemo(() => {
    if (!citations || citations.length === 0) return [];
    return [...citations]
      .sort((a, b) => b.relevance_score - a.relevance_score)
      .slice(0, 3);
  }, [citations]);

  return (
    <div
      className={`${styles.message} ${styles[role]}`}
      role="article"
      aria-label={`${role === 'user' ? 'Your' : 'Assistant'} message`}
    >
      <div className={styles.messageWrapper}>
        {/* Avatar */}
        <div className={styles.messageAvatar} aria-hidden="true">
          {role === 'user' ? (
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
              <circle cx="12" cy="7" r="4"></circle>
            </svg>
          ) : (
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <path d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253"></path>
            </svg>
          )}
        </div>

        {/* Message Bubble */}
        <div className={styles.messageBubble}>
          <div className={styles.messageContent}>
            {formattedContent}
            {isStreaming && <span className={styles.cursor} aria-hidden="true"></span>}
          </div>

          {/* Citations */}
          {topCitations.length > 0 && !isStreaming && (
            <div className={styles.citations} aria-label="Source citations">
              <div className={styles.citationsHeader}>
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="12"
                  height="12"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  aria-hidden="true"
                >
                  <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"></path>
                  <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"></path>
                </svg>
                Sources
              </div>
              <ul className={styles.citationsList}>
                {topCitations.map((citation, index) => (
                  <li
                    key={citation.chunk_id || index}
                    className={styles.citationItem}
                  >
                    <span className={styles.citationNumber}>{index + 1}</span>
                    <div className={styles.citationContent}>
                      <span className={styles.citationTitle}>
                        {citation.chapter_title}
                      </span>
                      {citation.section_title && (
                        <span className={styles.citationSection}>
                          {citation.section_title}
                        </span>
                      )}
                      {citation.excerpt && (
                        <span className={styles.citationExcerpt}>
                          "{citation.excerpt}"
                        </span>
                      )}
                    </div>
                    <span
                      className={`${styles.citationScore} ${getScoreClass(citation.relevance_score)}`}
                      title={`${Math.round(citation.relevance_score * 100)}% relevance`}
                    >
                      {Math.round(citation.relevance_score * 100)}%
                    </span>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
