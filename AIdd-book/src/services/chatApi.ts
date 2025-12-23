/**
 * Chat API service for communicating with the RAG chatbot backend.
 */

// Determine API base URL based on environment
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
  }
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
  }
  // Production: Use deployed backend URL
  // For GitHub Pages deployment, the backend needs to be deployed separately
  return 'https://e-book-physical-ai-humanoid-robotics.onrender.com';
}

const API_BASE_URL = getApiBaseUrl();

export interface ScopeContext {
  type: 'page' | 'selection' | 'module' | 'all';
  chapter_id?: string;
  module_id?: string;
  selected_text?: string;
}

export interface Citation {
  chunk_id: string;
  chapter_title: string;
  section_title: string;
  paragraph_number: number;
  relevance_score: number;
  excerpt: string;
}

export interface ChatRequest {
  session_id?: string;
  message: string;
  conversation_id?: string;
  context?: {
    module_filter?: string;
    locale?: string;
  };
}

// Legacy interface for internal use
interface LegacyChatRequest {
  session_id?: string;
  query: string;
  scope: ScopeContext;
}

export interface StreamEvent {
  type: 'content' | 'citation' | 'done' | 'error';
  text?: string;
  citation?: Citation;
  error?: string;
}

export interface Session {
  id: string;
  created_at: string;
  last_activity: string;
  conversation_history: Array<{
    query: string;
    response: string;
    timestamp: string;
  }>;
  scope_context: ScopeContext;
}

/**
 * Create a new session.
 */
export async function createSession(scopeContext?: ScopeContext): Promise<Session> {
  const response = await fetch(`${API_BASE_URL}/api/sessions`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: scopeContext ? JSON.stringify({ scope_context: scopeContext }) : '{}',
  });

  if (!response.ok) {
    throw new Error(`Failed to create session: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Get session by ID.
 */
export async function getSession(sessionId: string): Promise<Session | null> {
  const response = await fetch(`${API_BASE_URL}/api/sessions/${sessionId}`);

  if (response.status === 404) {
    return null;
  }

  if (!response.ok) {
    throw new Error(`Failed to get session: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Delete a session.
 */
export async function deleteSession(sessionId: string): Promise<void> {
  const response = await fetch(`${API_BASE_URL}/api/sessions/${sessionId}`, {
    method: 'DELETE',
  });

  if (!response.ok && response.status !== 404) {
    throw new Error(`Failed to delete session: ${response.statusText}`);
  }
}

/**
 * Send a chat message and receive streaming response.
 */
export async function sendChatMessage(
  request: LegacyChatRequest,
  onEvent: (event: StreamEvent) => void,
): Promise<void> {
  // Transform legacy request to backend ChatKit format
  const backendRequest: ChatRequest = {
    session_id: request.session_id,
    message: request.query,
    context: {
      module_filter: request.scope.type === 'module' ? request.scope.module_id :
                     request.scope.type === 'page' ? request.scope.chapter_id : undefined,
      locale: 'en',
    },
  };

  const response = await fetch(`${API_BASE_URL}/api/chatkit`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(backendRequest),
  });

  if (!response.ok) {
    let errorMessage = `Chat request failed: ${response.status} ${response.statusText}`;
    try {
      const errorText = await response.text();
      console.error('Backend error response:', errorText);
      const errorData = JSON.parse(errorText);
      errorMessage = errorData.detail || errorData.message || errorMessage;
    } catch (e) {
      console.error('Failed to parse error response:', e);
    }
    throw new Error(errorMessage);
  }

  if (!response.body) {
    throw new Error('No response body');
  }

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = '';
  let currentEvent = '';

  while (true) {
    const { done, value } = await reader.read();

    if (done) {
      break;
    }

    buffer += decoder.decode(value, { stream: true });

    // Process complete SSE events
    const lines = buffer.split('\n');
    buffer = lines.pop() || '';

    for (const line of lines) {
      if (line.startsWith('event: ')) {
        currentEvent = line.slice(7).trim();
      } else if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.slice(6));

          // Transform backend SSE format to frontend StreamEvent format
          if (currentEvent === 'message') {
            if (data.type === 'token') {
              // Backend sends "token" with "content", frontend expects "content" with "text"
              onEvent({ type: 'content', text: data.content });
            } else if (data.type === 'citation') {
              // Backend sends citation in "content" field
              onEvent({ type: 'citation', citation: data.content });
            }
          } else if (currentEvent === 'done') {
            onEvent({ type: 'done' });
          } else if (currentEvent === 'error') {
            onEvent({ type: 'error', error: data.error || 'An error occurred' });
          }

          currentEvent = '';
        } catch (e) {
          console.error('Failed to parse SSE event:', line, e);
        }
      }
    }
  }
}

/**
 * Get session ID from localStorage.
 */
export function getStoredSessionId(): string | null {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem('chatbot_session_id');
}

/**
 * Store session ID in localStorage.
 */
export function storeSessionId(sessionId: string): void {
  if (typeof window === 'undefined') return;
  localStorage.setItem('chatbot_session_id', sessionId);
}

/**
 * Clear stored session ID.
 */
export function clearStoredSessionId(): void {
  if (typeof window === 'undefined') return;
  localStorage.removeItem('chatbot_session_id');
}
