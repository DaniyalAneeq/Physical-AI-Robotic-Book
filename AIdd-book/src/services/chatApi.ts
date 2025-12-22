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
  request: ChatRequest,
  onEvent: (event: StreamEvent) => void,
): Promise<void> {
  const response = await fetch(`${API_BASE_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      ...(request.session_id && { 'X-Session-ID': request.session_id }),
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.message || `Chat request failed: ${response.statusText}`);
  }

  if (!response.body) {
    throw new Error('No response body');
  }

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = '';

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
      if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.slice(6));
          onEvent(data as StreamEvent);
        } catch (e) {
          // Ignore parse errors for malformed events
          console.warn('Failed to parse SSE event:', line);
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
