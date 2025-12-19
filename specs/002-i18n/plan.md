# Implementation Plan: Internationalization & Localization (English/Urdu)

**Branch**: `002-i18n` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-i18n/spec.md`

## Summary

Implement bilingual (English/Urdu) support across the Physical AI & Humanoid Robotics platform with:
- **Frontend**: Docusaurus native i18n with RTL layout and language toggle (auth-gated)
- **Backend**: FastAPI endpoints accepting `Accept-Language` header, locale-aware content retrieval
- **Data**: PostgreSQL user locale preferences, Qdrant dual-language payloads, translation cache
- **RAG**: Chatbot with automatic language detection, Urdu query translation, response caching

**Key Technical Decisions** (validated in [research.md](./research.md)):
- Docusaurus native i18n for UI (built-in RTL support)
- OpenAI gpt-4o-mini for translation (technical term handling + existing integration)
- PostgreSQL for translation cache (no new infrastructure)
- Single Qdrant collection with dual `content_en`/`content_ur` fields
- `langdetect` library for query language detection

---

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.6.2, React 19.0.0, Docusaurus 3.9.2
- Backend: Python 3.11+, FastAPI 0.115+

**Primary Dependencies**:
- Frontend: Docusaurus i18n plugin (built-in), React hooks
- Backend: `langdetect==1.0.9` (language detection), OpenAI SDK (translation)
- Database: PostgreSQL (Neon), Qdrant Client 1.10+

**Storage**:
- PostgreSQL: User locale preferences (`users.preferred_locale`), translation cache table
- Qdrant: Dual-language book content payloads (`content_en`, `content_ur`)

**Testing**:
- Frontend: Jest, React Testing Library, Playwright (E2E)
- Backend: pytest, pytest-asyncio, FastAPI TestClient

**Target Platform**:
- Frontend: Modern browsers (Chrome, Firefox, Safari, Edge) with RTL support
- Backend: Linux server (Docker container)

**Project Type**: Web application (Docusaurus frontend + FastAPI backend)

**Performance Goals** (from spec Success Criteria):
- Language toggle: < 2s UI transition
- Content retrieval: < 200ms (both locales)
- Chatbot translation overhead: < 500ms
- Translation cache hit rate: > 80%

**Constraints**:
- Must preserve existing English-only functionality (backwards compatible)
- RTL layout must not break code blocks or diagrams
- Translation API costs controlled via aggressive caching
- Authentication required for language toggle (Better Auth)

**Scale/Scope**:
- ~50-100 book modules to translate
- ~1000+ expected users (based on platform analytics)
- Support 2 languages initially (extensible to more)
- 24 functional requirements across 4 subsystems

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication Quality Gates (Section XIV)

- [x] **No custom auth logic bypassing Better Auth**
  - ✅ Locale preference stored in PostgreSQL, integrated with Better Auth user model
  - ✅ Language toggle gated behind `Depends(get_current_user)` (Better Auth session validation)
  - ✅ No custom JWT or session handling

- [x] **Secure cookie/token configuration**
  - ✅ No new cookies introduced; Better Auth handles all session management
  - ✅ Locale preference stored server-side, not in cookies

- [x] **Session expiration verified**
  - ✅ Locale preference persists beyond session; re-loaded from DB on login
  - ✅ No impact on Better Auth session expiration logic

- [x] **Frontend contains no secrets**
  - ✅ Translation API keys stored in backend `.env` only
  - ✅ OpenAI API calls made from FastAPI backend, never from frontend

- [x] **Auth flows tested in staging**
  - ⏳ Pending implementation (will be validated in Phase 6)

### RAG Chatbot Quality Gates (Section XV)

- [x] **Accuracy threshold**
  - ✅ Spec SC-004: 90%+ technical terminology accuracy (Urdu responses)
  - ✅ Technical terms preserved in English or with English equivalents
  - ✅ Validation: Manual review of 50 sample Urdu responses

- [x] **Latency threshold**
  - ✅ Spec SC-003: Content retrieval < 200ms
  - ✅ Spec PR-003: Chatbot translation < 500ms overhead
  - ✅ Mitigation: PostgreSQL translation cache (target 80% hit rate)

- [x] **Build requirements**
  - ✅ No breaking changes to existing English-only RAG pipeline
  - ✅ Backwards compatible: Defaults to English if locale not specified

- [x] **Accessibility**
  - ✅ RTL layout for Urdu (WCAG 2.1 compliance)
  - ✅ Language toggle accessible via keyboard navigation
  - ✅ Screen reader support: `lang` attribute on HTML elements

### Technology Currency (Section VI)

- [x] **Approved technologies used**
  - ✅ Docusaurus 3.9.2 (pinned in package.json)
  - ✅ FastAPI 0.115+ (existing dependency)
  - ✅ Qdrant Cloud (existing vector store)
  - ✅ Neon Serverless Postgres (existing database)
  - ✅ OpenAI SDK (existing for RAG)

- [x] **No unauthorized stack additions**
  - ✅ `langdetect==1.0.9` added (minimal, single-purpose library)
  - ✅ No Redis, no custom translation service, no new cloud dependencies

**Constitution Compliance**: ✅ **PASS** - All quality gates satisfied or pending implementation validation

---

## Project Structure

### Documentation (this feature)

```text
specs/002-i18n/
├── spec.md                # Feature specification (completed by /sp.specify)
├── plan.md                # This file (/sp.plan command output)
├── research.md            # Phase 0 output: Technology decisions
├── data-model.md          # Phase 1 output: Database schemas
├── quickstart.md          # Phase 1 output: Developer implementation guide
├── contracts/             # Phase 1 output: API specifications
│   └── api-spec.yaml      # OpenAPI 3.0 spec for i18n endpoints
├── checklists/            # Generated by /sp.specify
│   └── requirements.md    # Specification quality validation
└── tasks.md               # Phase 2 output: Actionable tasks (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
AIdd-book/                          # Docusaurus Frontend
├── docusaurus.config.js            # [MODIFY] Add i18n configuration
├── i18n/                           # [CREATE] Locale-specific translations
│   └── ur/                         # Urdu translations
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/            # (Static translations if any)
│       └── code.json               # UI element translations
├── src/
│   ├── components/
│   │   ├── LanguageToggle/         # [CREATE] Language switcher component
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── FallbackBanner/         # [CREATE] "Translation pending" indicator
│   │       ├── index.tsx
│   │       └── styles.module.css
│   ├── css/
│   │   └── rtl.css                 # [CREATE] RTL layout overrides
│   ├── hooks/
│   │   └── useLocale.tsx           # [CREATE] Custom hook for locale state
│   └── services/
│       └── localeApi.ts            # [CREATE] API calls for locale preferences
└── tests/
    └── e2e/
        └── i18n.spec.ts            # [CREATE] Playwright E2E tests

rag-chatbot/backend/                # FastAPI Backend
├── app/
│   ├── api/
│   │   ├── user.py                 # [MODIFY] Add locale preference endpoints
│   │   ├── content.py              # [MODIFY] Add locale-aware content retrieval
│   │   └── chatkit.py              # [MODIFY] Add translation to chatbot flow
│   ├── models/
│   │   └── user.py                 # [MODIFY] Add preferred_locale field
│   ├── services/
│   │   ├── translation.py          # [CREATE] Language detection & translation
│   │   └── translation_cache.py    # [CREATE] Cache management
│   └── utils/
│       └── locale.py               # [CREATE] Locale parsing & resolution
├── alembic/versions/
│   └── xxx_add_i18n_support.py     # [CREATE] Database migration
├── requirements.txt                # [MODIFY] Add langdetect==1.0.9
└── tests/
    └── integration/
        └── test_i18n_flow.py       # [CREATE] Integration tests

rag-chatbot/backend/scripts/        # Data ingestion & migration
├── update_qdrant_schema.py         # [CREATE] Add dual-language fields to Qdrant
├── export_qdrant_content.py        # [CREATE] Export English content for translation
├── bulk_translate.py               # [CREATE] (Optional) Automated translation via OpenAI
└── ingest_urdu_content.py          # [CREATE] Import translated Urdu content
```

**Structure Decision**: Web application structure (Option 2) is appropriate. Frontend and backend already separated per existing architecture. i18n feature adds components, services, and database migrations without restructuring.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations detected. All constitution requirements satisfied.*

---

## Implementation Phases

### Phase 0: Research & Technology Validation ✅

**Status**: Completed
**Output**: [research.md](./research.md)

**Key Decisions Made**:
1. **i18n Framework**: Docusaurus native i18n (built-in RTL, proven)
2. **Language Detection**: `langdetect` Python library (lightweight, sufficient accuracy)
3. **Translation Provider**: OpenAI gpt-4o-mini (technical term handling, existing integration)
4. **Translation Cache**: PostgreSQL (no new infrastructure, persistent)
5. **RTL Strategy**: Docusaurus auto-RTL + surgical CSS overrides
6. **Locale Storage**: PostgreSQL `users.preferred_locale` column
7. **Qdrant Schema**: Single collection with dual `content_en`/`content_ur` fields
8. **Locale Header**: Standard `Accept-Language` HTTP header

**Unknowns Resolved**: All 10 research questions answered (see research.md)

---

### Phase 1: Data Model & API Contracts ✅

**Status**: Completed
**Outputs**:
- [data-model.md](./data-model.md) - Entity schemas and relationships
- [contracts/api-spec.yaml](./contracts/api-spec.yaml) - OpenAPI specification
- [quickstart.md](./quickstart.md) - Developer implementation guide

**Artifacts Generated**:

1. **Data Model** ([data-model.md](./data-model.md)):
   - PostgreSQL: `users.preferred_locale` column (VARCHAR(5), DEFAULT 'en')
   - PostgreSQL: `translation_cache` table (query_hash, translations, metadata)
   - Qdrant: Dual-language payload schema (content_en, content_ur, translation_status)
   - Migration strategy: Non-destructive, backwards compatible

2. **API Contracts** ([contracts/api-spec.yaml](./contracts/api-spec.yaml)):
   - **GET /api/user/preferences**: Retrieve user locale preference
   - **PUT /api/user/preferences**: Update locale preference
   - **GET /api/content/{module_id}**: Locale-aware content retrieval (Accept-Language header)
   - **POST /api/chatkit/query**: Chatbot with translation support

3. **Quickstart Guide** ([quickstart.md](./quickstart.md)):
   - Week-by-week implementation timeline
   - Phase-by-phase tasks with validation checkpoints
   - Environment variables, monitoring, rollback procedures
   - Troubleshooting common issues

---

### Phase 2: Implementation Planning (This Document)

**Status**: In Progress
**Output**: This plan.md document

**Execution Order & Dependencies**:

```
┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: Database Schema Updates (Week 1)                  │
│ - PostgreSQL migration (users.preferred_locale, cache)     │
│ - Qdrant payload updates (content_ur, translation_status)  │
│ - Verification tests                                       │
└────────────────┬────────────────────────────────────────────┘
                 │ Dependency: Schema must exist before API/UI
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2: Frontend UI Localization (Week 2)                 │
│ - Docusaurus i18n config                                   │
│ - Language toggle component                                │
│ - Static UI translations (ur/code.json)                    │
│ - RTL CSS overrides                                        │
└────────────────┬────────────────────────────────────────────┘
                 │ Parallel with Backend (no blocking dependency)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 3: Backend API Updates (Week 2-3)                    │
│ - Locale helper functions                                  │
│ - User preferences endpoints (GET/PUT)                     │
│ - Content endpoint (locale-aware retrieval)                │
│ - Translation cache service                                │
└────────────────┬────────────────────────────────────────────┘
                 │ Dependency: Cache service needed for chatbot
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 4: RAG Chatbot Translation (Week 3)                  │
│ - Install langdetect library                               │
│ - Translation service (Urdu ↔ English)                     │
│ - Chatbot query handler updates                            │
│ - Technical term preservation logic                        │
└────────────────┬────────────────────────────────────────────┘
                 │ Dependency: Urdu content needed for testing
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 5: Content Ingestion & Translation (Week 4)          │
│ - Export English content from Qdrant                       │
│ - Translation workflow (professional or bulk OpenAI)       │
│ - Ingest Urdu content back to Qdrant                       │
│ - Verification (coverage, terminology, encoding)           │
└────────────────┬────────────────────────────────────────────┘
                 │ Dependency: All systems ready for E2E testing
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 6: Testing & Rollout (Week 4)                        │
│ - Integration tests (Python backend)                       │
│ - E2E tests (Playwright frontend)                          │
│ - Performance testing (latency, cache hit rate)            │
│ - Staged rollout (5% beta → 100% launch)                   │
└─────────────────────────────────────────────────────────────┘
```

**Critical Path**: Database → Backend API → RAG Translation → Content Ingestion → Testing

**Parallel Work**:
- Frontend UI (Week 2) can proceed in parallel with Backend API (Week 2-3)
- Integration tests can be written alongside feature development

---

## Migration Steps for Existing Content

### Current State: English-Only

- Qdrant collection `book_content` contains ~50-100 modules
- Each point has: `content_en`, `title_en`, `embedding` (based on English)
- No `content_ur`, `translation_status`, or `technical_terms` fields

### Migration Path: Non-Destructive Approach

**Step 1: Add New Fields (No Data Loss)**

```python
# Script: scripts/update_qdrant_schema.py
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_KEY)
collection_name = "book_content"

# Scroll through all points and update payloads
for point_batch in client.scroll(collection_name, limit=100):
    updated_points = []
    for point in point_batch[0]:
        # Add new fields without modifying existing ones
        point.payload["content_ur"] = None
        point.payload["title_ur"] = None
        point.payload["translation_status"] = "pending"
        point.payload["technical_terms"] = []
        updated_points.append(point)

    # Batch upsert (preserves embeddings and content_en)
    client.upsert(collection_name, points=updated_points)

print("✅ Qdrant schema updated. All English content preserved.")
```

**Step 2: Export for Translation**

```bash
python scripts/export_qdrant_content.py \
  --collection book_content \
  --output data/english_content.json \
  --format json
```

Output format:
```json
[
  {
    "module_id": "module-1-chapter-2",
    "title_en": "Introduction to PID Controllers",
    "content_en": "A PID controller is a control loop mechanism...",
    "technical_terms": ["PID Controller", "Actuator", "Feedback Loop"]
  },
  ...
]
```

**Step 3: Translation Workflow**

**Option A: Professional Translation** (Recommended)
- Send `english_content.json` to Urdu technical translator
- Provide glossary of terms to preserve (e.g., "PID Controller")
- Receive `urdu_content.json` with translated `title_ur` and `content_ur`

**Option B: Bulk OpenAI Translation** (Testing/Initial)
```bash
python scripts/bulk_translate.py \
  --input data/english_content.json \
  --output data/urdu_content.json \
  --target-lang ur \
  --preserve-terms data/technical_glossary.txt
```

**Step 4: Ingest Urdu Content**

```bash
python scripts/ingest_urdu_content.py \
  --input data/urdu_content.json \
  --collection book_content \
  --dry-run  # Verify first

# If dry-run looks good:
python scripts/ingest_urdu_content.py \
  --input data/urdu_content.json \
  --collection book_content
```

Script behavior:
- Match by `module_id`
- Update `content_ur`, `title_ur` fields
- Set `translation_status = 'complete'`
- Preserve `content_en` and `embedding` unchanged

**Step 5: Verification**

```python
# Verify coverage
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_KEY)

total = client.count(collection_name="book_content").count
complete = client.count(
    collection_name="book_content",
    count_filter={"must": [{"key": "translation_status", "match": {"value": "complete"}}]}
).count

print(f"Translation coverage: {(complete/total)*100:.1f}%")
# Target: > 80%
```

**Rollback**: If issues arise, Qdrant points can be reverted by re-upserting with `content_ur = None`.

---

## Caching & Performance Considerations

### Translation Cache Strategy

**Problem**: OpenAI translation API adds 500-1500ms latency per chatbot query

**Solution**: PostgreSQL-based translation cache with SHA-256 query hashing

**Implementation**:

```python
# File: app/services/translation_cache.py
import hashlib
from sqlalchemy.orm import Session
from app.models.translation_cache import TranslationCache

def compute_query_hash(query: str, src_lang: str, tgt_lang: str) -> str:
    """Compute cache key: SHA-256(normalized_query|src|tgt)"""
    normalized = query.lower().strip()
    normalized = ''.join(c for c in normalized if c.isalnum() or c.isspace())
    composite = f"{normalized}|{src_lang}|{tgt_lang}"
    return hashlib.sha256(composite.encode('utf-8')).hexdigest()

def get_cached_translation(query_hash: str, db: Session) -> str | None:
    """Retrieve cached translation. Update last_accessed and access_count."""
    cache_entry = db.query(TranslationCache).filter(
        TranslationCache.query_hash == query_hash
    ).first()

    if cache_entry:
        cache_entry.last_accessed = datetime.utcnow()
        cache_entry.access_count += 1
        db.commit()
        return cache_entry.translated_response

    return None

def cache_translation(
    query_hash: str,
    query: str,
    response: str,
    src_lang: str,
    tgt_lang: str,
    db: Session
) -> None:
    """Store new translation in cache."""
    cache_entry = TranslationCache(
        query_hash=query_hash,
        source_language=src_lang,
        target_language=tgt_lang,
        original_query=query,
        translated_response=response,
    )
    db.add(cache_entry)
    db.commit()
```

**Performance Targets**:
- Cache lookup: ~10-20ms (PostgreSQL indexed query)
- Cache miss + API call: ~500-1500ms
- Target cache hit rate: **80%+** after initial usage

**Cache Eviction**: No automatic eviction (translations don't expire). Optional future optimization: LRU eviction based on `last_accessed` if table grows > 100K rows.

### Qdrant Payload Size Management

**English-only payload**: ~100KB average per chunk
**Dual-language payload**: ~150KB average (50% increase due to Urdu text)

**Impact**: Qdrant efficiently stores payloads separately from vectors. No impact on vector search performance (embedding size unchanged at 1536 dimensions).

**Mitigation**: If storage costs become prohibitive, can lazy-load Urdu content (fetch from separate table only when `locale=ur`). Current approach preferred for simplicity.

### RTL Layout Performance

**Concern**: Does RTL CSS impact page load time?

**Measurement**:
- Baseline (English LTR): ~800ms page load
- With RTL (Urdu): Target < 880ms (< 10% degradation per spec PR-005)

**Optimization**:
- Docusaurus applies RTL via `[dir="rtl"]` attribute (no JavaScript overhead)
- CSS overrides for LTR code blocks are minimal (~50 lines)
- Font loading: Urdu uses system fonts (no web font download)

**Validation**: Lighthouse performance score must remain > 90 for both locales.

---

## Auth-Gated Urdu Toggle Rollout

### Requirement: Language Toggle Only for Logged-In Users

**Specification**: Users MUST be logged in (authenticated via Better Auth) to access the language toggle. Anonymous users see English-only content.

**Implementation**:

**Frontend (AIdd-book/src/components/LanguageToggle/index.tsx)**:

```tsx
import { useAuth } from '@/hooks/useAuth'; // Better Auth hook

export function LanguageToggle() {
  const { user, isAuthenticated } = useAuth();

  // Hide toggle if user not logged in
  if (!isAuthenticated) {
    return null;
  }

  return (
    <div className="language-toggle">
      <select
        value={currentLocale}
        onChange={(e) => handleLocaleChange(e.target.value)}
      >
        <option value="en">English</option>
        <option value="ur">اردو</option>
      </select>
    </div>
  );
}
```

**Backend (rag-chatbot/backend/app/api/user.py)**:

```python
from fastapi import Depends
from app.auth import get_current_user  # Better Auth dependency
from app.models.user import User

@router.put("/user/preferences")
async def update_preferences(
    data: UpdatePreferencesRequest,
    user: User = Depends(get_current_user),  # Auth gate
    db: Session = Depends(get_db)
):
    """Update user locale preference. Requires authentication."""
    if data.preferred_locale:
        user.preferred_locale = data.preferred_locale
        db.commit()

    return {"user_id": user.id, "preferred_locale": user.preferred_locale}
```

**Behavior**:
- Anonymous users: UI in English, no toggle visible
- Logged-in users: Toggle visible in navbar, can switch to Urdu
- Preference saved to database, persists across sessions

### Staged Rollout Strategy

**Week 4: Beta Launch (5% of Users)**

1. **Feature Flag** (environment variable):
   ```bash
   I18N_ENABLED=true
   I18N_BETA_PERCENTAGE=5
   ```

2. **Backend Logic** (app/utils/feature_flags.py):
   ```python
   def is_i18n_enabled_for_user(user_id: str) -> bool:
       if not os.getenv("I18N_ENABLED") == "true":
           return False

       beta_pct = int(os.getenv("I18N_BETA_PERCENTAGE", 0))
       # Deterministic hash-based sampling
       user_hash = int(hashlib.md5(user_id.encode()).hexdigest(), 16)
       return (user_hash % 100) < beta_pct
   ```

3. **Frontend Conditional Rendering**:
   ```tsx
   const { user } = useAuth();
   const i18nEnabled = await checkFeatureFlag(user.id, 'i18n');

   if (i18nEnabled) {
     return <LanguageToggle />;
   }
   return null;
   ```

**Monitoring During Beta**:
- Track locale adoption: How many beta users select Urdu?
- Monitor errors: Any crashes or 500s related to locale handling?
- Cache hit rate: Is translation cache performing as expected?
- User feedback: Survey beta users on translation quality

**Full Rollout Criteria**:
- Zero critical bugs in beta
- Cache hit rate > 60%
- Average Urdu query response time < 2s
- User satisfaction > 80% (beta survey)

**Week 5: Full Launch (100% of Users)**

Update environment variable:
```bash
I18N_BETA_PERCENTAGE=100
```

Remove feature flag checks (make permanent in codebase).

---

## Risks & Mitigations

### Risk 1: Poor Translation Quality for Technical Terms

**Impact**: Users confused by incorrect Urdu translations of "PID Controller", "Actuator", etc.

**Likelihood**: Medium (automated translation may not understand robotics context)

**Mitigation**:
1. **Glossary-Based Translation**: Provide OpenAI with technical term glossary
   ```python
   prompt = f"""Translate to Urdu. Preserve these terms in English: {technical_terms}.
   Format as: "Actuator (اییکچوایٹر)" with English in parentheses."""
   ```

2. **Professional Review**: Budget for human review of critical modules (Module 1-2)

3. **Fallback Indicator**: If translation quality poor, user can toggle back to English instantly

**Contingency**: If automated translation fails quality checks, delay Urdu content launch and prioritize professional translation.

---

### Risk 2: RTL Layout Breaks Complex UI Components

**Impact**: Code blocks, diagrams, tables render incorrectly in RTL mode

**Likelihood**: Medium (RTL is notoriously difficult for technical content)

**Mitigation**:
1. **Comprehensive Browser Testing**: Test matrix across Chrome, Firefox, Safari, Edge
2. **Surgical LTR Overrides**: Force LTR for code, diagrams
   ```css
   [dir="rtl"] pre, [dir="rtl"] code {
     direction: ltr !important;
     text-align: left !important;
   }
   ```
3. **Visual Regression Tests**: Playwright snapshots of key pages in both locales

**Contingency**: If RTL breaks are severe, provide "View in English" toggle for specific pages (temporary workaround).

---

### Risk 3: Translation API Costs Exceed Budget

**Impact**: Unexpected $500+ monthly OpenAI spend for chatbot translations

**Likelihood**: Low (cache should mitigate), but high impact if occurs

**Mitigation**:
1. **Aggressive Caching**: 80% hit rate target reduces API calls by 5x
2. **Cost Monitoring**: Daily alerts if spend > $20/day
3. **Rate Limiting**: Max 60 chatbot queries/user/hour (per spec)
4. **Fallback**: If costs spike, temporarily disable Urdu chatbot (keep UI localization)

**Contingency**: Implement spending cap in OpenAI dashboard ($100/month limit). If hit, queue queries for batch processing overnight (slower but cheaper).

---

### Risk 4: Qdrant Payload Size Causes Performance Degradation

**Impact**: Vector search slows down due to 50% larger payloads

**Likelihood**: Low (Qdrant designed for large payloads)

**Mitigation**:
1. **Benchmark Before/After**: Measure p95 search latency with English-only vs. dual-language
2. **Lazy Loading**: If needed, fetch Urdu content from separate table (not in vector payload)
3. **Payload Compression**: Qdrant supports compression (enable if needed)

**Contingency**: If search latency > 300ms, split into two collections (English, Urdu) with shared embeddings.

---

### Risk 5: Language Detection Fails for Mixed Queries

**Impact**: Query "ROS 2 میں navigation کیسے کام کرتا ہے؟" detected as English, response in wrong language

**Likelihood**: Medium (langdetect struggles with mixed-language text)

**Mitigation**:
1. **Confidence Threshold**: If langdetect confidence < 80%, fall back to user's preferred locale
   ```python
   detected = langdetect.detect_langs(query)
   if detected[0].prob < 0.8:
       language = user.preferred_locale  # Fallback
   ```
2. **User Preference Override**: Accept-Language header takes precedence over detection
3. **Manual Language Button**: Allow user to force language (e.g., "Ask in Urdu" button)

**Contingency**: If detection accuracy < 85%, disable auto-detection and require explicit language selection.

---

## Validation Checkpoints

### Checkpoint 1: Database Migration (End of Week 1)

**Criteria**:
- [ ] PostgreSQL: `users.preferred_locale` column exists, default 'en'
- [ ] PostgreSQL: `translation_cache` table created with indexes
- [ ] Qdrant: All points have `content_ur`, `title_ur`, `translation_status` fields
- [ ] Zero production errors during migration

**Validation**:
```sql
-- PostgreSQL
SELECT COUNT(*) FROM users WHERE preferred_locale IS NULL; -- Expect 0
SELECT COUNT(*) FROM translation_cache; -- Expect 0 (empty but exists)

-- Qdrant (Python)
assert all(p.payload.get("translation_status") == "pending" for p in qdrant_client.scroll(...)[0])
```

---

### Checkpoint 2: Frontend Localization (End of Week 2)

**Criteria**:
- [ ] Language toggle visible for logged-in users
- [ ] Selecting Urdu switches UI to RTL layout
- [ ] Code blocks remain LTR in RTL mode
- [ ] Static UI elements (nav, footer) translated to Urdu
- [ ] Browser compatibility: Chrome, Firefox, Safari, Edge

**Validation**:
```bash
# E2E test
npm run test:e2e -- i18n.spec.ts

# Manual test checklist:
# 1. Login → Toggle visible ✓
# 2. Select "اردو" → UI flips to RTL ✓
# 3. View code example → LTR preserved ✓
# 4. Logout → Toggle hidden ✓
```

---

### Checkpoint 3: Backend API (End of Week 3)

**Criteria**:
- [ ] GET `/api/user/preferences` returns `preferred_locale`
- [ ] PUT `/api/user/preferences` persists locale to database
- [ ] GET `/api/content/{module_id}` with `Accept-Language: ur` returns Urdu content
- [ ] Translation cache stores and retrieves queries correctly

**Validation**:
```python
# Integration test
response = client.put("/api/user/preferences", json={"preferred_locale": "ur"}, headers=auth_headers)
assert response.json()["preferred_locale"] == "ur"

response = client.get("/api/content/module-1-chapter-1", headers={"Accept-Language": "ur", **auth_headers})
assert response.json()["locale"] == "ur"
```

---

### Checkpoint 4: RAG Chatbot (End of Week 3)

**Criteria**:
- [ ] English query returns English response
- [ ] Urdu query returns Urdu response
- [ ] Technical terms preserved (e.g., "PID Controller" not translated)
- [ ] Repeat query served from cache (`translation_cached: true`)
- [ ] Latency < 2s for cache miss, < 500ms for cache hit

**Validation**:
```python
# Test 1: Urdu query
response = client.post("/api/chatkit/query", json={"query": "PID کیا ہے؟", "session_id": "test_123"})
assert response.json()["response_language"] == "ur"
assert "PID Controller" in response.json()["response"]  # Term preserved

# Test 2: Cache hit
response2 = client.post("/api/chatkit/query", json={"query": "PID کیا ہے؟", "session_id": "test_456"})
assert response2.json()["translation_cached"] == True
assert response2.json()["processing_time_ms"] < 500
```

---

### Checkpoint 5: Content Translation (End of Week 4)

**Criteria**:
- [ ] At least 80% of modules have `translation_status = 'complete'`
- [ ] Urdu content renders correctly (no encoding issues)
- [ ] Technical terms preserved in sample review (50 modules)

**Validation**:
```python
# Coverage check
from qdrant_client import QdrantClient
client = QdrantClient(...)

total = client.count(collection_name="book_content").count
complete = client.count(
    collection_name="book_content",
    count_filter={"must": [{"key": "translation_status", "match": {"value": "complete"}}]}
).count

assert (complete / total) >= 0.80, f"Coverage {(complete/total)*100:.1f}% < 80%"
```

---

### Checkpoint 6: Performance & Launch (End of Week 4)

**Criteria** (from spec Success Criteria):
- [ ] **SC-001**: Language toggle < 2s
- [ ] **SC-002**: 95% UI elements correct in Urdu
- [ ] **SC-003**: Content retrieval < 200ms
- [ ] **SC-004**: 90%+ chatbot terminology accuracy
- [ ] **SC-005**: 80% cache hit rate
- [ ] **SC-006**: 100% preference persistence
- [ ] **SC-009**: 95% RTL browser consistency
- [ ] **SC-010**: 90%+ mixed-query accuracy

**Validation**:
```bash
# Performance tests
npm run test:performance -- i18n-latency.spec.ts
pytest tests/performance/test_i18n_latency.py

# Manual QA:
# - Test on 5 devices (desktop, mobile, tablet)
# - Test on 4 browsers (Chrome, Firefox, Safari, Edge)
# - Survey 10 beta users for satisfaction
```

---

## Next Steps

### Immediate Actions (Before /sp.tasks)

1. **Review Artifacts**:
   - [ ] Read [research.md](./research.md) for technology decisions
   - [ ] Read [data-model.md](./data-model.md) for database schemas
   - [ ] Read [quickstart.md](./quickstart.md) for week-by-week guide
   - [ ] Review [contracts/api-spec.yaml](./contracts/api-spec.yaml) for API details

2. **Update Agent Context** (see section below)

3. **Generate Tasks**: Run `/sp.tasks` to decompose this plan into actionable tasks

### Agent Context Update (Phase 1 Completion)

Update `.claude/settings.local.json` with new technologies introduced in this feature:

**New Technologies Added**:
- langdetect==1.0.9 (Python language detection)
- Docusaurus i18n plugin (built-in, version 3.9.2)
- OpenAI translation (using existing SDK, new use case)

**Active Technologies (unchanged)**:
- Python 3.11+ (Backend)
- FastAPI 0.115+ (Backend framework)
- PostgreSQL (Neon) - Extended with i18n tables
- Qdrant Client 1.10+ - Extended with dual-language payloads
- TypeScript 5.6.2 (Frontend)
- React 19.0.0 (Frontend UI)
- Docusaurus 3.9.2 (Frontend framework)
- Better Auth (Authentication)

---

## Summary

This implementation plan provides:

✅ **Phased Approach**: 6 clear phases (Database → Frontend → Backend → RAG → Content → Testing)
✅ **Execution Order**: Dependencies mapped (Database first, Testing last)
✅ **Migration Strategy**: Non-destructive Qdrant and PostgreSQL migrations
✅ **Caching Strategy**: PostgreSQL translation cache with 80% target hit rate
✅ **Auth Integration**: Language toggle gated behind Better Auth
✅ **Risk Mitigation**: 5 major risks identified with contingency plans
✅ **Validation**: 6 checkpoints with measurable criteria

**Ready for**: `/sp.tasks` command to generate actionable task list

---

**Last Updated**: 2025-12-17
**Status**: ✅ Complete - Ready for Task Generation
