# Data Model: Integrated RAG Chatbot

**Feature Branch**: `003-rag-chatbot`
**Date**: 2025-12-10

## Entity Definitions

### 1. ContentChunk

A segment of textbook content stored as a vector embedding with metadata for retrieval and citation.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | UUID, primary key |
| module_id | string | Module identifier | e.g., "module-1", "module-2" |
| chapter_id | string | Chapter identifier | e.g., "chapter-3" |
| section_id | string | Section identifier | e.g., "section-3.2" |
| paragraph_id | integer | Paragraph number within section | >= 1 |
| content_type | enum | Type of content | "paragraph", "code_block", "heading" |
| text | string | Original text content | Max 2000 chars |
| file_path | string | Source markdown file path | Relative to docs/ |
| vector | float[1536] | Embedding vector | OpenAI text-embedding-3-small |

**Stored in**: Qdrant Cloud (vectors + payload)

**Relationships**:
- Belongs to one Module (via module_id)
- Belongs to one Chapter (via chapter_id)
- Belongs to one Section (via section_id)

---

### 2. Session

A user's conversation context for multi-turn interactions.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Unique session identifier | Primary key, auto-generated |
| created_at | timestamp | Session creation time | Auto-set, UTC |
| last_activity | timestamp | Last query timestamp | Updated on each query, UTC |
| conversation_history | JSON array | Previous Q&A pairs | Max 20 entries |
| scope_context | JSON object | Current scope selection | See ScopeContext schema |

**Stored in**: Neon Serverless Postgres

**Schema (SQL)**:
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    conversation_history JSONB DEFAULT '[]',
    scope_context JSONB DEFAULT '{}'
);

CREATE INDEX idx_sessions_last_activity ON sessions(last_activity);
```

**Lifecycle**:
- Created: On first query from new browser session
- Updated: On each query (last_activity, conversation_history)
- Deleted: After 24 hours of inactivity (cleanup job)

---

### 3. Query

A user's natural language question with scope constraints.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| session_id | UUID | Reference to session | Required |
| query_text | string | User's question | 1-1000 chars |
| scope | ScopeContext | Retrieval scope | Required |
| timestamp | timestamp | Query submission time | Auto-set, UTC |

**Note**: Queries are ephemeral (processed and logged, not persisted as separate entity).

---

### 4. ScopeContext

Defines the boundaries for content retrieval.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| type | enum | Scope type | "page", "selection", "module", "all" |
| module_id | string | Module filter | Required if type="module" |
| chapter_id | string | Chapter filter | Required if type="page" |
| selected_text | string | User-highlighted text | Required if type="selection" |

**Examples**:
```json
// Current page scope
{"type": "page", "chapter_id": "chapter-3"}

// Selected text scope
{"type": "selection", "selected_text": "ROS 2 navigation stack configuration"}

// Module scope
{"type": "module", "module_id": "module-2"}

// All content scope
{"type": "all"}
```

---

### 5. Response

The chatbot's answer with source citations.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| session_id | UUID | Reference to session | Required |
| content | string | Generated answer text | Markdown formatted |
| citations | Citation[] | Source references | At least 1 required |
| latency_ms | integer | Response generation time | Measured server-side |
| timestamp | timestamp | Response generation time | Auto-set, UTC |

**Note**: Responses are streamed via SSE and logged for analytics.

---

### 6. Citation

A reference to source content in the textbook.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| chunk_id | string | Reference to ContentChunk | Required |
| chapter_title | string | Human-readable chapter name | e.g., "ROS 2 Navigation" |
| section_title | string | Human-readable section name | e.g., "Configuring Nav2" |
| paragraph_number | integer | Paragraph within section | >= 1 |
| relevance_score | float | Similarity score from Qdrant | 0.0 - 1.0 |
| excerpt | string | Short text excerpt | Max 200 chars |

**Example**:
```json
{
  "chunk_id": "abc-123-def",
  "chapter_title": "ROS 2 Navigation",
  "section_title": "Configuring Nav2",
  "paragraph_number": 4,
  "relevance_score": 0.92,
  "excerpt": "The navigation stack requires a properly configured..."
}
```

---

### 7. QueryLog

Analytics record for queries (anonymized).

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Unique log identifier | Primary key, auto-generated |
| session_id | UUID | Reference to session | No FK constraint (privacy) |
| query_text | string | User's question | Stored for analytics |
| scope_type | string | Scope type used | "page", "selection", "module", "all" |
| response_latency_ms | integer | End-to-end latency | Milliseconds |
| chunk_count | integer | Chunks retrieved | Number of Qdrant results |
| created_at | timestamp | Log creation time | Auto-set, UTC |

**Stored in**: Neon Serverless Postgres

**Schema (SQL)**:
```sql
CREATE TABLE query_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID,  -- No FK constraint for privacy
    query_text TEXT NOT NULL,
    scope_type VARCHAR(50),
    response_latency_ms INTEGER,
    chunk_count INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_query_logs_created_at ON query_logs(created_at);
```

**Privacy Note**: No PII stored. Session IDs are random UUIDs not linked to user identity.

---

## Entity Relationship Diagram

```
┌─────────────────┐
│   Session       │
│─────────────────│
│ id (PK)         │
│ created_at      │
│ last_activity   │
│ conversation_   │
│   history       │
│ scope_context   │
└────────┬────────┘
         │
         │ 1:N (via session_id)
         │
         ▼
┌─────────────────┐
│   QueryLog      │
│─────────────────│
│ id (PK)         │
│ session_id      │
│ query_text      │
│ scope_type      │
│ response_       │
│   latency_ms    │
│ chunk_count     │
│ created_at      │
└─────────────────┘


┌─────────────────────────────────────────────────────────┐
│                    Qdrant Collection                     │
│                   "textbook_content"                     │
│─────────────────────────────────────────────────────────│
│                                                          │
│  ┌──────────────────┐                                   │
│  │  ContentChunk    │                                   │
│  │──────────────────│                                   │
│  │ id (point_id)    │                                   │
│  │ vector[1536]     │                                   │
│  │ payload:         │                                   │
│  │   module_id      │                                   │
│  │   chapter_id     │                                   │
│  │   section_id     │                                   │
│  │   paragraph_id   │                                   │
│  │   content_type   │                                   │
│  │   text           │                                   │
│  │   file_path      │                                   │
│  └──────────────────┘                                   │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## State Transitions

### Session Lifecycle

```
[New Visit] ──create──▶ [Active] ──query──▶ [Active]
                            │                   │
                            │                   │
                       (24h idle)          (24h idle)
                            │                   │
                            ▼                   ▼
                       [Expired] ◀──────── [Expired]
                            │
                            │
                       (cleanup)
                            │
                            ▼
                       [Deleted]
```

### Query Processing Flow

```
[Received] ──validate──▶ [Valid] ──embed──▶ [Embedded]
    │                       │                    │
    │                       │                    │
(invalid)              (rate limit)          (search)
    │                       │                    │
    ▼                       ▼                    ▼
[Error 400]            [Error 429]          [Retrieved]
                                                 │
                                                 │
                                            (generate)
                                                 │
                                                 ▼
                                            [Streaming]
                                                 │
                                                 │
                                             (complete)
                                                 │
                                                 ▼
                                            [Logged]
```

## Validation Rules

### Session
- `conversation_history` max 20 entries (FIFO eviction)
- `scope_context` must match ScopeContext schema

### Query
- `query_text` length: 1-1000 characters
- `query_text` must not be empty or whitespace-only
- `scope.type` must be valid enum value

### ContentChunk
- `text` max 2000 characters
- `vector` must have exactly 1536 dimensions
- `module_id` must match existing module pattern
- `content_type` must be valid enum value

### Response
- `citations` must have at least 1 entry
- `content` must not be empty
- `relevance_score` must be between 0.0 and 1.0
