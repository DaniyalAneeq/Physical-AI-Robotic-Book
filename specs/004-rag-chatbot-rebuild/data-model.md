# Data Model: RAG Chatbot Full Rebuild

**Feature**: 004-rag-chatbot-rebuild
**Date**: 2025-12-12

---

## Entity Relationship Diagram

```
┌─────────────────────┐       ┌─────────────────────┐
│    Conversation     │       │      Message        │
├─────────────────────┤       ├─────────────────────┤
│ id (UUID, PK)       │───┐   │ id (UUID, PK)       │
│ session_id (String) │   │   │ conversation_id (FK)│◄──┘
│ title (String?)     │   │   │ role (String)       │
│ created_at (DateTime)│   └──│ content (Text)      │
│ updated_at (DateTime)│       │ citations (JSON?)   │
└─────────────────────┘       │ created_at (DateTime)│
                              └─────────────────────┘

┌─────────────────────┐       ┌─────────────────────────┐
│     QueryLog        │       │   TextbookChunk (Qdrant)│
├─────────────────────┤       ├─────────────────────────┤
│ id (UUID, PK)       │       │ id (UUID)               │
│ query_text (Text)   │       │ vector (1536 dims)      │
│ response_summary    │       │ payload:                │
│   (Text?)           │       │   - text (String)       │
│ latency_ms (Int?)   │       │   - module (String?)    │
│ created_at (DateTime)│       │   - chapter (String)    │
└─────────────────────┘       │   - section (String)    │
                              │   - file_path (String)  │
                              └─────────────────────────┘
```

---

## PostgreSQL Entities (Neon)

### Conversation

Represents a chat session identified by browser-generated UUID.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, default uuid4 | Unique conversation identifier |
| `session_id` | VARCHAR(255) | NOT NULL, INDEX | Browser localStorage UUID |
| `title` | VARCHAR(255) | NULL | Auto-generated or user-set title |
| `created_at` | TIMESTAMP | NOT NULL, default now() | Creation timestamp |
| `updated_at` | TIMESTAMP | NULL, on update | Last activity timestamp |

**Indexes**:
- `idx_conversation_session_id` on `session_id` (for session lookups)
- `idx_conversation_created_at` on `created_at` (for retention cleanup)

**Constraints**:
- Conversations older than 7 days are eligible for cleanup

---

### Message

Individual message within a conversation.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, default uuid4 | Unique message identifier |
| `conversation_id` | UUID | FK → Conversation.id, NOT NULL | Parent conversation |
| `role` | VARCHAR(50) | NOT NULL, CHECK in ('user', 'assistant') | Message author |
| `content` | TEXT | NOT NULL | Message content |
| `citations` | TEXT | NULL | JSON array of citation objects |
| `created_at` | TIMESTAMP | NOT NULL, default now() | Creation timestamp |

**Indexes**:
- `idx_message_conversation_id` on `conversation_id` (for conversation retrieval)
- `idx_message_created_at` on `created_at` (for ordering)

**Foreign Keys**:
- `fk_message_conversation` → `conversations(id)` ON DELETE CASCADE

**Citations JSON Schema**:
```json
[
  {
    "module": "module-1",
    "chapter": "01-introduction",
    "section": "Getting Started",
    "file_path": "docs/module-1/01-introduction.md",
    "relevance_score": 0.92
  }
]
```

---

### QueryLog

Audit log for debugging and analytics (anonymized).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, default uuid4 | Unique log identifier |
| `query_text` | TEXT | NOT NULL | User query (no PII) |
| `response_summary` | TEXT | NULL | First 200 chars of response |
| `latency_ms` | INTEGER | NULL | Query-to-response time in ms |
| `chunks_retrieved` | INTEGER | NULL | Number of chunks from Qdrant |
| `created_at` | TIMESTAMP | NOT NULL, default now() | Log timestamp |

**Indexes**:
- `idx_querylog_created_at` on `created_at` (for analytics queries)

**Notes**:
- No session_id stored to ensure anonymity
- Retention: 30 days (for analytics, longer than conversations)

---

## Qdrant Vector Entity

### TextbookChunk

Vector representation of textbook content chunk.

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Deterministic from file_path + section hash |
| `vector` | Float[1536] | text-embedding-3-small embedding |

**Payload Fields**:

| Field | Type | Indexed | Description |
|-------|------|---------|-------------|
| `text` | String | No | Raw chunk text content |
| `module` | String | Yes (keyword) | Module identifier (e.g., "module-1") |
| `chapter` | String | Yes (keyword) | Chapter filename without extension |
| `section` | String | No | Section heading |
| `file_path` | String | No | Relative path from docs/ |
| `char_count` | Integer | No | Character count for analytics |
| `created_at` | String | No | ISO timestamp of indexing |

**Collection Configuration**:
```python
{
    "collection_name": "textbook_chunks",
    "vectors_config": {
        "size": 1536,
        "distance": "Cosine"
    },
    "optimizers_config": {
        "default_segment_number": 2
    }
}
```

**Payload Index**:
```python
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="module",
    field_schema=models.PayloadSchemaType.KEYWORD
)
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="chapter",
    field_schema=models.PayloadSchemaType.KEYWORD
)
```

---

## State Transitions

### Conversation Lifecycle

```
[New Session] → Created (empty)
     │
     ▼
[First Message] → Active (has messages)
     │
     ├── [User sends message] → Active (more messages)
     │
     ├── [7 days inactive] → Expired (eligible for cleanup)
     │
     └── [User deletes] → Deleted (cascade to messages)
```

### Message States

Messages are immutable once created. No state transitions.

---

## Validation Rules

### Conversation
- `session_id` must be valid UUID v4 format
- `title` max length 255 characters
- `title` auto-generated from first message if not set

### Message
- `role` must be one of: `user`, `assistant`
- `content` cannot be empty
- `citations` must be valid JSON array if present

### TextbookChunk
- `text` max length ~6000 characters (~1500 tokens)
- `module` must match pattern `module-\d+` or be null
- `file_path` must be valid relative path under `docs/`

---

## Cleanup Jobs

### Conversation Retention (7 days)

```sql
-- Run daily via cron/scheduler
DELETE FROM conversations
WHERE created_at < NOW() - INTERVAL '7 days';
-- Messages cascade-deleted via FK
```

### QueryLog Retention (30 days)

```sql
-- Run weekly
DELETE FROM query_logs
WHERE created_at < NOW() - INTERVAL '30 days';
```

---

## Migration Strategy

### Initial Migration (Alembic)

```python
# alembic/versions/001_initial.py
def upgrade():
    op.create_table(
        'conversations',
        sa.Column('id', sa.UUID(), primary_key=True),
        sa.Column('session_id', sa.String(255), nullable=False),
        sa.Column('title', sa.String(255), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.func.now()),
        sa.Column('updated_at', sa.DateTime(), onupdate=sa.func.now()),
    )
    op.create_index('idx_conversation_session_id', 'conversations', ['session_id'])
    op.create_index('idx_conversation_created_at', 'conversations', ['created_at'])

    op.create_table(
        'messages',
        sa.Column('id', sa.UUID(), primary_key=True),
        sa.Column('conversation_id', sa.UUID(), sa.ForeignKey('conversations.id', ondelete='CASCADE')),
        sa.Column('role', sa.String(50), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('citations', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.func.now()),
    )
    op.create_index('idx_message_conversation_id', 'messages', ['conversation_id'])

    op.create_table(
        'query_logs',
        sa.Column('id', sa.UUID(), primary_key=True),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('response_summary', sa.Text(), nullable=True),
        sa.Column('latency_ms', sa.Integer(), nullable=True),
        sa.Column('chunks_retrieved', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.func.now()),
    )
    op.create_index('idx_querylog_created_at', 'query_logs', ['created_at'])

def downgrade():
    op.drop_table('query_logs')
    op.drop_table('messages')
    op.drop_table('conversations')
```
