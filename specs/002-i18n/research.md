# Research: i18n Implementation for AI-Powered Book Platform

**Feature**: 002-i18n
**Date**: 2025-12-17
**Phase**: Phase 0 - Research & Technology Validation

## Purpose

Validate technology choices and resolve unknowns for implementing bilingual (English/Urdu) support across Docusaurus frontend, FastAPI backend, and RAG chatbot with RTL layout requirements.

---

## Research Areas

### 1. Docusaurus i18n Plugin Capabilities

**Question**: Can Docusaurus native i18n handle RTL languages with dynamic locale switching?

**Research Findings**:
- Docusaurus 3.x includes built-in i18n plugin with RTL support via `direction` config
- Locale switching requires page reload (not SPA-style instant switch)
- Configuration in `docusaurus.config.js`:
  ```js
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: { direction: 'rtl' }
    }
  }
  ```
- Static content translations stored in `i18n/{locale}/` directory structure
- Dynamic content (from APIs) requires custom handling

**Decision**: Use Docusaurus native i18n for UI shell; implement custom locale state management for dynamic book content

**Rationale**:
- Native plugin handles RTL CSS automatically
- Reduces custom CSS complexity
- Well-documented and battle-tested

**Alternatives Considered**:
- Custom i18n solution → Rejected (reinventing the wheel, higher maintenance)
- Next.js i18n routing → Not applicable (locked into Docusaurus per constitution)

---

### 2. Language Detection for Chatbot Queries

**Question**: What library should detect language (English vs Urdu) in chatbot queries?

**Research Findings**:
- **langdetect** (Python): Fast, supports 55 languages including Urdu
  - Accuracy: 95%+ for sentences > 20 characters
  - Limitation: Lower accuracy for short queries (< 10 words)
- **fasttext** (Facebook): Higher accuracy but larger model size (170MB)
- **polyglot**: Requires large dependencies (NumPy, ICU)

**Decision**: Use `langdetect` library for query language detection

**Rationale**:
- Lightweight (no heavy dependencies)
- Sufficient accuracy for typical chatbot queries
- Fallback strategy: If confidence < 80%, default to user's preferred locale from session

**Alternatives Considered**:
- fasttext → Rejected (overkill for 2-language scenario, large model)
- Manual detection via Unicode ranges → Rejected (unreliable for mixed-language queries)

---

### 3. Translation API Choice

**Question**: Which translation service for on-demand Urdu ↔ English translation?

**Research Findings**:
- **OpenAI GPT-4**: Excellent technical term handling, context-aware
  - Cost: ~$0.01 per 1K tokens input, ~$0.03 per 1K tokens output
  - Latency: 500-1500ms for typical query
- **Google Translate API**: Fast but literal translations, poor technical context
  - Cost: $20 per 1M characters
  - Latency: 100-300ms
- **DeepL API**: High quality but limited Urdu support (not in free tier)

**Decision**: Use OpenAI GPT-4 for translation with aggressive caching

**Rationale**:
- Already using OpenAI for RAG (same API key, consolidated billing)
- Superior technical term handling critical for robotics content
- Can instruct to preserve specific terms (e.g., "PID Controller")
- Caching mitigates latency and cost concerns

**Alternatives Considered**:
- Google Translate → Rejected (poor technical accuracy, may translate "Actuator" incorrectly)
- DeepL → Rejected (no Urdu support in accessible tiers)

---

### 4. Translation Cache Strategy

**Question**: Should translation cache use PostgreSQL or Redis?

**Research Findings**:
- **PostgreSQL** (existing Neon instance):
  - Pros: Already provisioned, persistent, queryable
  - Cons: Higher latency than Redis (~10-20ms vs 1-2ms)
  - Storage: Unlimited in practical terms
- **Redis**:
  - Pros: Sub-millisecond lookups, built-in TTL
  - Cons: Requires new infrastructure, memory limits, ephemeral

**Decision**: Use PostgreSQL for translation cache with indexed query_hash lookup

**Rationale**:
- 10-20ms cache hit latency acceptable vs. 500-1500ms API call
- No new infrastructure (adheres to constitution's technology currency)
- Persistent cache survives restarts
- Can analyze cache hit rates via SQL queries

**Alternatives Considered**:
- Redis → Rejected (new dependency, operational overhead)
- In-memory dict → Rejected (lost on restart, no cross-instance sharing)

**Implementation Details**:
```sql
CREATE TABLE translation_cache (
  id SERIAL PRIMARY KEY,
  query_hash VARCHAR(64) UNIQUE NOT NULL,
  source_language VARCHAR(5) NOT NULL,
  target_language VARCHAR(5) NOT NULL,
  original_query TEXT NOT NULL,
  translated_response TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  access_count INTEGER DEFAULT 1
);
CREATE INDEX idx_translation_cache_hash ON translation_cache(query_hash);
```

---

### 5. RTL Layout Strategy

**Question**: How to handle RTL layout for complex components (code blocks, diagrams)?

**Research Findings**:
- Docusaurus applies `[dir="rtl"]` to root HTML element when Urdu locale active
- CSS logical properties (`margin-inline-start`) auto-flip in RTL
- Code blocks MUST remain LTR (technical content convention)
- Override approach:
  ```css
  [dir="rtl"] pre, [dir="rtl"] code {
    direction: ltr;
    text-align: left;
  }
  ```

**Decision**: Use Docusaurus auto-RTL with explicit LTR overrides for technical content

**Rationale**:
- Leverages Docusaurus built-in RTL handling
- Surgical overrides only where needed (code, diagrams)
- Maintains text directionality correctness per Unicode Bidirectional Algorithm

**Alternatives Considered**:
- Manual RTL CSS for every component → Rejected (error-prone, maintenance burden)
- JavaScript-based dynamic class toggling → Rejected (Docusaurus already handles this)

**Test Cases**:
- Verify code blocks remain LTR within RTL page
- Verify mixed Urdu text + English terms render correctly
- Browser compatibility: Chrome, Firefox, Safari, Edge

---

### 6. Locale Preference Storage

**Question**: Where to store user's preferred locale (PostgreSQL vs Better Auth session)?

**Research Findings**:
- **PostgreSQL users table**: Persistent, queryable, simple
  - Schema: `ALTER TABLE users ADD COLUMN preferred_locale VARCHAR(5) DEFAULT 'en'`
- **Better Auth session metadata**: Session-scoped, requires session API extension
  - May require custom Better Auth adapter

**Decision**: Store in PostgreSQL `users.preferred_locale` column; sync to session on login

**Rationale**:
- Explicit data model aligns with specification Key Entities
- Survives session expiration
- Can default new users to `en` via DB constraint
- Simple to query for analytics

**Alternatives Considered**:
- Cookie-based preference → Rejected (not tied to user account, lost on device change)
- Better Auth metadata only → Rejected (lost on session expiry, complicates migration)

---

### 7. Qdrant Dual-Language Payload Strategy

**Question**: Should we use separate collections for English/Urdu or single collection with both fields?

**Research Findings**:
- **Single collection with dual fields**:
  - Pros: Easier content synchronization, single embedding per semantic chunk
  - Cons: Larger payload size (~150% vs English-only)
- **Separate collections**:
  - Pros: Smaller per-query payloads
  - Cons: Duplication of embeddings, complex sync logic

**Decision**: Single Qdrant collection with `content_en` and `content_ur` fields

**Rationale**:
- Embeddings only computed once (on English text)
- Urdu content retrieved from same document payload
- Simpler vector search logic (one collection to query)
- Payload size increase acceptable (< 200KB per chunk typical)

**Schema**:
```json
{
  "content_en": "English text...",
  "content_ur": "Urdu text...",
  "title_en": "Title",
  "title_ur": "عنوان",
  "translation_status": "complete|pending|partial",
  "embedding": [...]
}
```

---

### 8. Fallback Indicator UX

**Question**: How to display fallback when Urdu translation unavailable?

**Research Findings**:
- Best practices from bilingual sites (Wikipedia, MDN):
  - Inline banner: "This content is not available in Urdu. Showing English version."
  - Styling: Subtle background color, not blocking
- Avoid: Modal dialogs (disruptive), hiding content (confusing)

**Decision**: Inline banner at top of content with `translation_status: pending|partial|unavailable`

**Implementation**:
```tsx
{translationStatus !== 'complete' && locale === 'ur' && (
  <div className="translation-notice">
    <Icon /> اردو ترجمہ دستیاب نہیں - انگریزی مواد دکھایا جا رہا ہے
    (Urdu translation unavailable - showing English content)
  </div>
)}
```

**Rationale**:
- Clear communication to user
- Non-blocking (content still accessible)
- Bilingual notice (Urdu + English) ensures comprehension

---

### 9. Performance Optimization: Chatbot Translation Caching

**Question**: What cache key strategy ensures high hit rate?

**Research Findings**:
- **Query hash function**: SHA-256 of normalized query
  - Normalization: lowercase, trim whitespace, remove punctuation
  - Example: "PID کیا ہے؟" → "pid کیا ہے" → hash
- **Cache hit analysis**: Expect ~40-60% hit rate for common questions
  - "What is PID controller" likely asked multiple times
  - Specific debugging questions likely unique

**Decision**: Cache key = SHA-256(normalized_query + source_lang + target_lang)

**Rationale**:
- Normalization increases hit rate for case/punctuation variants
- Language pair in key prevents en→ur and ur→en collisions
- 64-character hash fits VARCHAR(64) index efficiently

**Cache Invalidation**:
- No expiration (translations don't change)
- Optional: Manual purge if content updated

---

### 10. Locale Header Handling

**Question**: Should we use `Accept-Language` header or custom `X-Locale` header?

**Research Findings**:
- **Accept-Language**: Standard HTTP header, browser auto-sends
  - Format: `Accept-Language: ur-PK, en-US;q=0.9`
  - Requires parsing quality values
- **Custom X-Locale**: Explicit control, simpler parsing
  - Format: `X-Locale: ur`
  - No browser auto-send, requires manual header addition

**Decision**: Use standard `Accept-Language` header with fallback to user preference

**Rationale**:
- Follows HTTP standards
- Works with browser language settings
- FastAPI has built-in parsing: `request.headers.get("accept-language")`

**Implementation**:
```python
def get_user_locale(request, user_id):
    # Priority: explicit header > user pref > default
    header_locale = parse_accept_language(request.headers.get("accept-language"))
    user_pref = db.get_user_locale(user_id)
    return header_locale or user_pref or "en"
```

---

## Technology Stack Summary

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Frontend i18n | Docusaurus native plugin | Built-in RTL, proven solution |
| Language Detection | `langdetect` (Python) | Lightweight, sufficient accuracy |
| Translation API | OpenAI GPT-4 | Technical term handling, existing integration |
| Translation Cache | PostgreSQL (Neon) | No new infra, persistent, queryable |
| RTL Layout | Docusaurus auto + CSS overrides | Minimal custom code |
| Locale Storage | PostgreSQL `users.preferred_locale` | Persistent, simple schema |
| Vector Store | Qdrant single collection | Shared embeddings, simpler sync |
| Locale Header | `Accept-Language` standard | HTTP compliance, browser support |

---

## Dependencies & Prerequisites

### New Python Dependencies
```txt
langdetect==1.0.9  # Language detection for chatbot queries
```

### Database Migrations
1. PostgreSQL: Add `preferred_locale` column to users table
2. PostgreSQL: Create `translation_cache` table
3. Qdrant: Add `content_ur`, `title_ur`, `translation_status` fields to existing collection

### Frontend Dependencies
- No new NPM packages required (Docusaurus i18n is built-in)

---

## Risk Mitigations Validated

| Risk | Mitigation Strategy | Validation |
|------|---------------------|------------|
| Translation API cost overrun | Aggressive PostgreSQL caching with indexed lookups | Expect 80%+ hit rate after initial usage |
| Low language detection accuracy | Fallback to user preferred locale if confidence < 80% | `langdetect` confidence score available |
| RTL layout breaks | Explicit LTR overrides for code/diagrams, comprehensive browser testing | Test matrix: Chrome/FF/Safari/Edge |
| Latency from translation API | Cache-first strategy, async translation where possible | Max 500ms overhead per spec |

---

## Open Questions Resolved

1. ✅ i18n framework choice → Docusaurus native
2. ✅ Language detection → langdetect library
3. ✅ Translation provider → OpenAI GPT-4
4. ✅ Cache storage → PostgreSQL
5. ✅ RTL handling → Auto with overrides
6. ✅ Locale preference → PostgreSQL users table
7. ✅ Qdrant schema → Single collection, dual fields
8. ✅ Fallback UX → Inline banner
9. ✅ Cache key strategy → SHA-256 normalized query
10. ✅ Locale header → Accept-Language standard

---

## Next Steps

Proceed to **Phase 1: Data Model & Contracts** with all technology decisions finalized.
