# Data Model: i18n & Localization

**Feature**: 002-i18n
**Date**: 2025-12-17
**Phase**: Phase 1 - Data Model Design

## Overview

Data model for bilingual (English/Urdu) support across user preferences, book content, and translation caching.

---

## Entity Relationship Diagram

```
┌─────────────────┐
│     User        │
│─────────────────│
│ id (PK)         │
│ email           │
│ preferred_locale│──┐
└─────────────────┘  │
                     │ 1:1
                     │
┌─────────────────┐  │
│TranslationCache │  │
│─────────────────│  │
│ id (PK)         │  │
│ query_hash (UK) │  │
│ source_language │  │
│ target_language │  │
│ original_query  │  │
│ translated_resp │  │
│ created_at      │  │
│ last_accessed   │  │
│ access_count    │  │
└─────────────────┘  │
                     │
┌──────────────────────────────┐
│  LocalizedBookModule (Qdrant)│
│──────────────────────────────│
│ id (PK)                      │
│ content_en (TEXT)            │
│ content_ur (TEXT)            │
│ title_en (TEXT)              │
│ title_ur (TEXT)              │
│ module (STR)                 │
│ chapter (STR)                │
│ translation_status (ENUM)    │
│ technical_terms (ARRAY)      │
│ embedding (VECTOR)           │
│ metadata (JSON)              │
└──────────────────────────────┘
```

---

## Entities

### 1. User (PostgreSQL Extension)

**Purpose**: Store user language preference persistently

**Schema Extension**:
```sql
-- Migration: Add locale preference to existing users table
ALTER TABLE users
ADD COLUMN preferred_locale VARCHAR(5) DEFAULT 'en' NOT NULL;

-- Index for analytics queries
CREATE INDEX idx_users_locale ON users(preferred_locale);

-- Constraint: Only allow supported locales
ALTER TABLE users
ADD CONSTRAINT chk_locale CHECK (preferred_locale IN ('en', 'ur'));
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `preferred_locale` | VARCHAR(5) | NOT NULL, DEFAULT 'en', CHECK IN ('en', 'ur') | User's preferred UI/content language |

**Relationships**:
- Belongs to existing `users` table managed by Better Auth

**Validation Rules**:
- Locale must be one of: `en`, `ur`
- Default to `en` for new users
- Updated when user changes language via toggle

**State Transitions**:
```
New User → preferred_locale = 'en'
User toggles language → preferred_locale updates to selected value
User resets preferences → preferred_locale = 'en'
```

---

### 2. TranslationCache (PostgreSQL)

**Purpose**: Cache chatbot query translations to reduce API costs and latency

**Schema**:
```sql
CREATE TABLE translation_cache (
  id SERIAL PRIMARY KEY,
  query_hash VARCHAR(64) UNIQUE NOT NULL,
  source_language VARCHAR(5) NOT NULL,
  target_language VARCHAR(5) NOT NULL,
  original_query TEXT NOT NULL,
  translated_response TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
  last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
  access_count INTEGER DEFAULT 1 NOT NULL
);

-- Indexes
CREATE INDEX idx_translation_cache_hash ON translation_cache(query_hash);
CREATE INDEX idx_translation_cache_languages ON translation_cache(source_language, target_language);
CREATE INDEX idx_translation_cache_accessed ON translation_cache(last_accessed);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | SERIAL | PRIMARY KEY | Auto-increment ID |
| `query_hash` | VARCHAR(64) | UNIQUE, NOT NULL | SHA-256 hash of normalized query + lang pair |
| `source_language` | VARCHAR(5) | NOT NULL, CHECK IN ('en', 'ur') | Language of original query |
| `target_language` | VARCHAR(5) | NOT NULL, CHECK IN ('en', 'ur') | Language of response |
| `original_query` | TEXT | NOT NULL | Verbatim user query |
| `translated_response` | TEXT | NOT NULL | Cached translation result |
| `created_at` | TIMESTAMP | DEFAULT NOW(), NOT NULL | Cache entry creation time |
| `last_accessed` | TIMESTAMP | DEFAULT NOW(), NOT NULL | Last cache hit time |
| `access_count` | INTEGER | DEFAULT 1, NOT NULL | Number of cache hits |

**Relationships**:
- No foreign keys (standalone cache table)
- Queried by `query_hash` for lookups

**Validation Rules**:
- `query_hash` computed as: `SHA256(normalize(query) + source_lang + target_lang)`
- Normalization: lowercase, trim whitespace, remove non-alphanumeric except spaces
- Languages must be in `('en', 'ur')`

**Indexes**:
- `query_hash`: Fast cache lookups (UNIQUE constraint provides B-tree index)
- `source_language, target_language`: Analytics queries
- `last_accessed`: Cache eviction policies (future optimization)

**Lifecycle**:
```
Query arrives → Compute hash → Check cache
  ├─ Cache hit → Update last_accessed, increment access_count, return cached response
  └─ Cache miss → Call translation API → Insert new entry → Return response
```

---

### 3. LocalizedBookModule (Qdrant)

**Purpose**: Store book content with both English and Urdu translations in vector database

**Schema** (Qdrant Payload):
```json
{
  "id": "module-1-chapter-2-section-3",
  "content_en": "A PID controller is a control loop mechanism...",
  "content_ur": "PID کنٹرولر ایک کنٹرول لوپ میکانزم ہے...",
  "title_en": "Introduction to PID Controllers",
  "title_ur": "PID کنٹرولرز کا تعارف",
  "module": "module-1",
  "chapter": "chapter-2",
  "section": "section-3",
  "translation_status": "complete",
  "technical_terms": ["PID Controller", "Actuator", "Feedback Loop"],
  "embedding": [0.123, 0.456, ...],
  "metadata": {
    "last_updated_en": "2025-12-01T00:00:00Z",
    "last_updated_ur": "2025-12-10T00:00:00Z",
    "translator": "professional",
    "word_count_en": 450,
    "word_count_ur": 520
  }
}
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | STRING | PRIMARY KEY | Unique identifier: `{module}-{chapter}-{section}` |
| `content_en` | TEXT | NOT NULL | Full English content text |
| `content_ur` | TEXT | NULLABLE | Full Urdu content text (may be pending) |
| `title_en` | STRING | NOT NULL | English title/heading |
| `title_ur` | STRING | NULLABLE | Urdu title/heading |
| `module` | STRING | NOT NULL | Module identifier (e.g., `module-1`) |
| `chapter` | STRING | NOT NULL | Chapter identifier (e.g., `chapter-2`) |
| `section` | STRING | NULLABLE | Section identifier (granular content chunks) |
| `translation_status` | ENUM | NOT NULL, DEFAULT 'pending' | One of: `complete`, `pending`, `partial`, `unavailable` |
| `technical_terms` | ARRAY[STRING] | NULLABLE | List of technical terms to preserve in English |
| `embedding` | VECTOR(1536) | NOT NULL | OpenAI text-embedding-3-small vector |
| `metadata` | JSON | NULLABLE | Additional metadata (timestamps, translator info) |

**Relationships**:
- No relational foreign keys (vector database)
- Hierarchical structure via `module`, `chapter`, `section` fields

**Validation Rules**:
- `content_en` always present (source of truth for embeddings)
- `content_ur` may be NULL if `translation_status != 'complete'`
- `translation_status` values:
  - `complete`: Both en and ur content exist
  - `pending`: Urdu translation not started
  - `partial`: Urdu translation incomplete or low quality
  - `unavailable`: Urdu translation impossible (e.g., code-only content)
- `embedding` always computed from `content_en` only (single source)

**Indexing** (Qdrant):
- Vector index on `embedding` (HNSW)
- Payload indexes on: `module`, `chapter`, `translation_status`

**Lifecycle**:
```
Content ingestion:
1. English content arrives → Generate embedding → Store with translation_status='pending'
2. Urdu translation added → Update content_ur, set translation_status='complete'
3. Content updated → Re-embed English, mark Urdu as 'partial' if not re-translated

Content retrieval:
1. User requests content in locale → Query Qdrant
2. Return content_en or content_ur based on locale
3. If locale=ur and content_ur is NULL → Return content_en with fallback indicator
```

---

## Field-Level Validation

### Locale Values
All locale fields MUST use ISO 639-1 codes:
- `en`: English
- `ur`: Urdu

Future extensibility: VARCHAR(5) supports regional variants (e.g., `en-US`, `ur-PK`) if needed.

### Translation Status Values
Enum constrained to:
- `complete`: Full translation available
- `pending`: Translation not yet started
- `partial`: Translation exists but incomplete or needs review
- `unavailable`: Cannot be translated (code snippets, diagrams without captions)

### Query Hash Computation
```python
import hashlib

def compute_query_hash(query: str, source_lang: str, target_lang: str) -> str:
    """Compute cache key for translation query."""
    normalized = query.lower().strip()
    # Remove punctuation except spaces
    normalized = ''.join(c for c in normalized if c.isalnum() or c.isspace())
    composite_key = f"{normalized}|{source_lang}|{target_lang}"
    return hashlib.sha256(composite_key.encode('utf-8')).hexdigest()
```

---

## Data Migration Strategy

### Phase 1: Schema Updates (Non-Destructive)

**PostgreSQL**:
```sql
-- Step 1: Add locale column (allows NULL initially)
ALTER TABLE users ADD COLUMN preferred_locale VARCHAR(5);

-- Step 2: Backfill existing users with default
UPDATE users SET preferred_locale = 'en' WHERE preferred_locale IS NULL;

-- Step 3: Apply constraint
ALTER TABLE users ALTER COLUMN preferred_locale SET NOT NULL;
ALTER TABLE users ALTER COLUMN preferred_locale SET DEFAULT 'en';
ALTER TABLE users ADD CONSTRAINT chk_locale CHECK (preferred_locale IN ('en', 'ur'));

-- Step 4: Create index
CREATE INDEX idx_users_locale ON users(preferred_locale);

-- Step 5: Create translation cache table
-- (New table, no migration needed)
CREATE TABLE translation_cache (...);
```

**Qdrant**:
```python
# Step 1: Retrieve existing collection
client = QdrantClient(...)
collection_name = "book_content"

# Step 2: Update points with new fields (preserves existing data)
for point in client.scroll(collection_name):
    point.payload["content_ur"] = None
    point.payload["title_ur"] = None
    point.payload["translation_status"] = "pending"
    point.payload["technical_terms"] = []
    client.upsert(collection_name, points=[point])
```

### Phase 2: Content Translation Ingestion

**Process**:
1. Export English content from Qdrant
2. Professional translators produce Urdu versions
3. Ingest Urdu content via batch update script
4. Update `translation_status` to `complete`

**Rollback Plan**:
- PostgreSQL: Drop `preferred_locale` column (non-breaking for existing functionality)
- Qdrant: Remove new payload fields (existing en content unaffected)

---

## Performance Considerations

### Translation Cache

**Expected Performance**:
- Cache lookup: ~10-20ms (PostgreSQL indexed query)
- Cache miss + API call: ~500-1500ms (OpenAI latency)
- Cache hit rate: Target 80%+ after initial usage

**Optimization**:
- Index on `query_hash` ensures O(log n) lookup
- Periodically analyze `access_count` to identify popular queries
- Consider materialized views for analytics

### Qdrant Payload Size

**Impact Analysis**:
- English-only payload: ~100KB average per chunk
- Dual-language payload: ~150KB average (50% increase)
- Urdu text typically 10-15% longer than English (more verbose grammar)

**Mitigation**:
- Qdrant efficiently stores payloads separately from vectors
- No impact on vector search performance (embedding size unchanged)
- Storage cost increase: ~50% (acceptable per spec PR-006)

---

## Security Considerations

### Translation Cache

**Privacy**:
- No user-identifying information stored in cache
- Query text may contain sensitive debugging info → Accept risk (transient data)
- No PII expected in technical queries

**Access Control**:
- Cache table not directly exposed via API
- Backend-only access, queries sanitized before hashing

### User Locale Preference

**Storage**:
- Stored in PostgreSQL alongside Better Auth user data
- No encryption needed (non-sensitive preference data)
- GDPR compliance: Included in user data export/deletion workflows

---

## Testing Strategy

### Data Integrity Tests

1. **Locale Constraint**: Attempt to set `preferred_locale='fr'` → Should fail
2. **Translation Status**: Verify only valid enum values accepted
3. **Cache Hash Uniqueness**: Verify no collisions for distinct queries

### Migration Tests

1. **Backwards Compatibility**: Existing users get `preferred_locale='en'` by default
2. **Qdrant Update**: Verify English content unchanged after adding Urdu fields
3. **Index Performance**: Benchmark locale lookup < 20ms

---

## Monitoring & Observability

### Metrics to Track

1. **Cache Hit Rate**: `SUM(access_count) / COUNT(*)` for translation_cache
2. **Locale Distribution**: Count of users per `preferred_locale`
3. **Translation Coverage**: `COUNT(*) WHERE translation_status='complete'` in Qdrant
4. **Cache Growth Rate**: New entries per day in translation_cache

### Alerts

- Cache hit rate drops below 60% → Investigate common queries
- Qdrant payload size exceeds 200KB average → Review content chunking
- Translation API latency > 2s p95 → Check OpenAI service status

---

## Appendix: Example Queries

### Get User Locale Preference
```sql
SELECT preferred_locale FROM users WHERE id = $1;
```

### Cache Lookup
```sql
SELECT translated_response, access_count
FROM translation_cache
WHERE query_hash = $1
LIMIT 1;
```

### Cache Hit (Update Stats)
```sql
UPDATE translation_cache
SET last_accessed = NOW(), access_count = access_count + 1
WHERE query_hash = $1;
```

### Cache Miss (Insert New)
```sql
INSERT INTO translation_cache (query_hash, source_language, target_language, original_query, translated_response)
VALUES ($1, $2, $3, $4, $5)
ON CONFLICT (query_hash) DO NOTHING;
```

### Qdrant Retrieval (Urdu Content)
```python
results = client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=5
)

for result in results:
    locale = "ur"  # From user preference
    content = result.payload.get(f"content_{locale}") or result.payload["content_en"]
    translation_status = result.payload["translation_status"]
    # Return content + fallback indicator if needed
```

---

**Status**: ✅ Complete - Ready for Phase 2 (API Contracts)
