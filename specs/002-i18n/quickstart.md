# Quickstart: i18n Implementation

**Feature**: 002-i18n
**Audience**: Developers implementing the i18n feature
**Estimated Time**: 3-4 weeks (phased rollout)

## Overview

This quickstart guides you through implementing bilingual (English/Urdu) support for the Physical AI & Humanoid Robotics platform, covering frontend UI localization, backend API changes, data model updates, and RAG chatbot translation.

---

## Prerequisites

Before starting implementation:

- [ ] Read `spec.md` - Feature specification
- [ ] Read `research.md` - Technology decisions and rationale
- [ ] Read `data-model.md` - Database schema changes
- [ ] Read `contracts/api-spec.yaml` - API endpoint specifications
- [ ] Access to:
  - [ ] Neon PostgreSQL database (write permissions)
  - [ ] Qdrant Cloud instance (admin access)
  - [ ] OpenAI API key (for translations)
  - [ ] Better Auth configuration
- [ ] Development environment:
  - [ ] Node.js 18+ (for Docusaurus frontend)
  - [ ] Python 3.11+ (for FastAPI backend)
  - [ ] Git access to repository

---

## Implementation Phases

### Phase 1: Database Schema Updates (Week 1)

**Goal**: Prepare data layer for bilingual content

**Tasks**:
1. **PostgreSQL Migration** (Day 1-2)
   ```bash
   cd rag-chatbot/backend

   # Create migration file
   alembic revision -m "add_i18n_support"

   # Edit migration file with:
   # - Add users.preferred_locale column
   # - Create translation_cache table
   # - Add indexes

   # Run migration
   alembic upgrade head
   ```

   Migration script location: `rag-chatbot/backend/alembic/versions/xxx_add_i18n_support.py`

2. **Qdrant Schema Update** (Day 3-4)
   ```python
   # Update script: scripts/update_qdrant_schema.py
   from qdrant_client import QdrantClient

   client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_KEY)

   # Add new payload fields to existing points
   # See data-model.md for full schema
   ```

3. **Verification** (Day 5)
   - [ ] Run `SELECT preferred_locale FROM users LIMIT 1;` → Expect 'en'
   - [ ] Run `SELECT * FROM translation_cache LIMIT 1;` → Expect empty table
   - [ ] Query Qdrant: Verify `content_ur`, `translation_status` fields exist

**Validation Checkpoint**:
- All existing users have `preferred_locale = 'en'`
- Qdrant English content unchanged
- No production downtime

---

### Phase 2: Frontend UI Localization (Week 2)

**Goal**: Implement Docusaurus i18n with RTL support and language toggle

**Tasks**:

1. **Configure Docusaurus i18n** (Day 1)
   ```bash
   cd AIdd-book

   # Create Urdu locale directory
   mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
   mkdir -p i18n/ur

   # Update docusaurus.config.js
   ```

   Add to `docusaurus.config.js`:
   ```javascript
   module.exports = {
     i18n: {
       defaultLocale: 'en',
       locales: ['en', 'ur'],
       localeConfigs: {
         en: {
           label: 'English',
           direction: 'ltr',
           htmlLang: 'en-US',
         },
         ur: {
           label: 'اردو',
           direction: 'rtl',
           htmlLang: 'ur-PK',
         },
       },
     },
     // ... rest of config
   };
   ```

2. **Create Language Toggle Component** (Day 2)
   ```bash
   # Create component
   touch src/components/LanguageToggle/index.tsx
   touch src/components/LanguageToggle/styles.module.css
   ```

   Component location: `AIdd-book/src/components/LanguageToggle/`
   - Dropdown: English | اردو
   - Calls PUT `/api/user/preferences` to save preference
   - Triggers Docusaurus locale switch via `useDocusaurusContext()`

3. **Translate Static UI Elements** (Day 3-4)
   ```bash
   # Create UI translations file
   touch i18n/ur/code.json
   ```

   Translate in `i18n/ur/code.json`:
   - Navigation labels (Home, Modules, About, etc.)
   - Button text (Login, Logout, Next, Previous, etc.)
   - Footer content

4. **RTL CSS Overrides** (Day 5)
   ```bash
   # Add RTL-specific styles
   touch src/css/rtl.css
   ```

   Override in `src/css/rtl.css`:
   ```css
   /* Code blocks remain LTR */
   [dir="rtl"] pre,
   [dir="rtl"] code {
     direction: ltr;
     text-align: left;
   }

   /* Diagrams remain LTR */
   [dir="rtl"] .diagram-container {
     direction: ltr;
   }
   ```

**Validation Checkpoint**:
- [ ] Language toggle appears in navbar (logged-in users only)
- [ ] Clicking "اردو" switches UI to Urdu with RTL layout
- [ ] Code examples remain LTR within RTL pages
- [ ] Browser test: Chrome, Firefox, Safari, Edge

---

### Phase 3: Backend API Updates (Week 2-3)

**Goal**: Add locale-aware endpoints and locale resolution logic

**Tasks**:

1. **Locale Helper Functions** (Day 1)
   ```bash
   touch rag-chatbot/backend/app/utils/locale.py
   ```

   Implement:
   - `parse_accept_language(header: str) -> str | None`
   - `get_user_locale(user_id: str, db: Session, request: Request) -> str`

2. **Update User Preferences Endpoint** (Day 2)
   ```python
   # File: rag-chatbot/backend/app/api/user.py

   @router.get("/preferences")
   async def get_preferences(user: User = Depends(get_current_user)):
       return {
           "user_id": user.id,
           "preferred_locale": user.preferred_locale,
           # ... other preferences
       }

   @router.put("/preferences")
   async def update_preferences(
       data: UpdatePreferencesRequest,
       user: User = Depends(get_current_user),
       db: Session = Depends(get_db)
   ):
       if data.preferred_locale:
           user.preferred_locale = data.preferred_locale
       db.commit()
       return {"user_id": user.id, "preferred_locale": user.preferred_locale}
   ```

3. **Update Content Endpoint** (Day 3)
   ```python
   # File: rag-chatbot/backend/app/api/content.py

   @router.get("/content/{module_id}")
   async def get_content(
       module_id: str,
       request: Request,
       user: User = Depends(get_current_user),
       db: Session = Depends(get_db)
   ):
       locale = get_user_locale(user.id, db, request)

       # Query Qdrant
       result = qdrant_client.retrieve(
           collection_name="book_content",
           ids=[module_id]
       )[0]

       # Select content based on locale
       content_key = f"content_{locale}"
       content = result.payload.get(content_key) or result.payload["content_en"]

       return {
           "module_id": module_id,
           "locale": locale,
           "content": content,
           "translation_status": result.payload["translation_status"],
           # ... metadata
       }
   ```

4. **Add Translation Cache Service** (Day 4-5)
   ```bash
   touch rag-chatbot/backend/app/services/translation_cache.py
   ```

   Implement:
   - `compute_query_hash(query, src_lang, tgt_lang) -> str`
   - `get_cached_translation(query_hash) -> str | None`
   - `cache_translation(query_hash, query, response, src, tgt) -> None`

**Validation Checkpoint**:
- [ ] GET `/api/user/preferences` returns `preferred_locale`
- [ ] PUT `/api/user/preferences` with `preferred_locale: "ur"` persists to DB
- [ ] GET `/api/content/module-1-chapter-1` with `Accept-Language: ur` returns Urdu content (if available)
- [ ] API tests pass (use contracts/api-spec.yaml for reference)

---

### Phase 4: RAG Chatbot Translation (Week 3)

**Goal**: Integrate language detection, translation, and caching into chatbot flow

**Tasks**:

1. **Install Language Detection** (Day 1)
   ```bash
   cd rag-chatbot/backend
   pip install langdetect==1.0.9
   pip freeze > requirements.txt
   ```

2. **Create Translation Service** (Day 2)
   ```bash
   touch app/services/translation.py
   ```

   Implement:
   - `detect_language(query: str) -> str` (uses langdetect)
   - `translate_urdu_to_english(query: str) -> str` (OpenAI API)
   - `translate_english_to_urdu(response: str, preserve_terms: list[str]) -> str`

3. **Update Chatbot Query Handler** (Day 3-4)
   ```python
   # File: app/api/chatkit.py

   @router.post("/query")
   async def chatbot_query(
       data: ChatQueryRequest,
       user: User = Depends(get_current_user),
       db: Session = Depends(get_db)
   ):
       # 1. Detect language
       detected_lang = data.query_language or detect_language(data.query)

       # 2. Check cache
       cache_key = compute_query_hash(data.query, detected_lang, detected_lang)
       cached = get_cached_translation(cache_key)
       if cached:
           return {"response": cached, "translation_cached": True, ...}

       # 3. Translate query if Urdu
       search_query = data.query
       if detected_lang == "ur":
           search_query = translate_urdu_to_english(data.query)

       # 4. Vector search (always on English embeddings)
       results = qdrant_client.search(
           collection_name="book_content",
           query_vector=embed_text(search_query),
           limit=5
       )

       # 5. Generate response
       context = [r.payload["content_en"] for r in results]
       response = generate_response(context, search_query, language=detected_lang)

       # 6. Cache translation
       cache_translation(cache_key, data.query, response, detected_lang, detected_lang)

       return {"response": response, "translation_cached": False, ...}
   ```

4. **Technical Term Preservation** (Day 5)
   - Extract `technical_terms` from Qdrant payloads
   - Pass to translation service with instruction: "Preserve these terms in English: {terms}"

**Validation Checkpoint**:
- [ ] English query "What is a PID controller?" returns English answer
- [ ] Urdu query "PID کیا ہے؟" returns Urdu answer with "PID Controller" preserved in English
- [ ] Repeat query served from cache (check `translation_cached: true`)
- [ ] Latency < 2s for cache miss, < 500ms for cache hit

---

### Phase 5: Content Ingestion & Translation (Week 4)

**Goal**: Populate Qdrant with Urdu translations of book modules

**Tasks**:

1. **Export English Content** (Day 1)
   ```bash
   python scripts/export_qdrant_content.py --collection book_content --output english_content.json
   ```

2. **Translation Workflow** (Day 2-3)
   - **Option A**: Manual professional translation
     - Send `english_content.json` to translators
     - Receive `urdu_content.json`
   - **Option B**: Bulk OpenAI translation (for testing)
     ```bash
     python scripts/bulk_translate.py --input english_content.json --output urdu_content.json --target-lang ur
     ```

3. **Ingest Urdu Content** (Day 4)
   ```bash
   python scripts/ingest_urdu_content.py --input urdu_content.json
   ```

   Script updates Qdrant payloads:
   - Set `content_ur` from translation
   - Set `translation_status = 'complete'`
   - Preserve `content_en` and `embedding` unchanged

4. **Verify Content** (Day 5)
   ```python
   # Spot-check 10 random modules
   for module_id in random_sample:
       point = qdrant_client.retrieve(collection_name="book_content", ids=[module_id])[0]
       assert point.payload["content_ur"] is not None
       assert point.payload["translation_status"] == "complete"
       print(f"✓ {module_id}")
   ```

**Validation Checkpoint**:
- [ ] At least 80% of modules have `translation_status = 'complete'`
- [ ] Urdu content renders correctly in frontend (no encoding issues)
- [ ] Technical terms preserved (spot-check: "PID Controller", "Actuator", "ROS 2")

---

### Phase 6: Testing & Rollout (Week 4)

**Goal**: Comprehensive testing and staged rollout

**Tasks**:

1. **Integration Tests** (Day 1-2)
   ```bash
   # Run test suite
   cd rag-chatbot/backend
   pytest tests/integration/test_i18n_flow.py -v
   ```

   Test scenarios:
   - User changes locale → preference persists across sessions
   - Urdu content request → Returns Urdu if available, English if not
   - Chatbot Urdu query → Response in Urdu with cached translation
   - Mixed-language query → Correct detection and response

2. **Frontend E2E Tests** (Day 3)
   ```bash
   cd AIdd-book
   npm run test:e2e
   ```

   Test:
   - Language toggle switches UI and layout
   - RTL layout correct (navigation, content, buttons)
   - Code blocks remain LTR in RTL mode
   - Browser compatibility (Chrome, Firefox, Safari, Edge)

3. **Performance Testing** (Day 4)
   - [ ] Language toggle < 2s transition time
   - [ ] Content retrieval < 200ms (both locales)
   - [ ] Chatbot translation overhead < 500ms
   - [ ] Cache hit rate > 60% (after 100 test queries)

4. **Staged Rollout** (Day 5)
   - **Beta**: Enable for 5% of users (feature flag)
   - Monitor:
     - Locale distribution (% choosing Urdu)
     - Cache hit rate
     - Translation API costs
     - User feedback
   - **Full Rollout**: If metrics meet targets, enable for 100%

**Validation Checkpoint**:
- [ ] All integration tests pass
- [ ] All E2E tests pass
- [ ] Performance meets spec (SC-001 through SC-010)
- [ ] Zero production incidents during beta

---

## Environment Variables

Add to `.env` files:

**Backend (`rag-chatbot/backend/.env`)**:
```bash
# Translation
OPENAI_API_KEY=sk-...  # Already exists
TRANSLATION_CACHE_ENABLED=true
TRANSLATION_DEFAULT_LOCALE=en

# Locale
SUPPORTED_LOCALES=en,ur
```

**Frontend (`AIdd-book/.env`)**:
```bash
# i18n (handled by Docusaurus, no additional env vars needed)
```

---

## Monitoring & Alerts

Set up monitoring for:

1. **Locale Adoption**
   ```sql
   SELECT preferred_locale, COUNT(*) FROM users GROUP BY preferred_locale;
   ```
   → Alert if Urdu adoption < 5% after 1 week (may indicate UX issue)

2. **Translation Cache Performance**
   ```sql
   SELECT
     COUNT(*) as total_queries,
     AVG(access_count) as avg_hits_per_query,
     SUM(access_count) / COUNT(*) as cache_hit_rate
   FROM translation_cache;
   ```
   → Alert if cache hit rate < 60%

3. **Translation API Costs**
   - Monitor OpenAI API spend
   - Alert if daily cost > $50 (may indicate cache failure or excessive usage)

4. **Content Coverage**
   ```python
   from qdrant_client import QdrantClient
   client = QdrantClient(...)

   total = client.count(collection_name="book_content")
   complete = client.count(
       collection_name="book_content",
       count_filter={"must": [{"key": "translation_status", "match": {"value": "complete"}}]}
   )
   coverage = (complete / total) * 100
   print(f"Urdu coverage: {coverage:.1f}%")
   ```
   → Alert if coverage drops below 75%

---

## Rollback Plan

If critical issues arise:

1. **Frontend Rollback** (< 5 min)
   ```bash
   cd AIdd-book
   # Comment out i18n config in docusaurus.config.js
   # Hide language toggle component
   npm run build && npm run deploy
   ```

2. **Backend Rollback** (< 10 min)
   ```bash
   # Disable locale endpoints
   # Comment out Accept-Language parsing
   # Redeploy backend
   ```

3. **Database Rollback** (< 30 min)
   ```sql
   -- Revert users table (optional, no harm if left)
   ALTER TABLE users DROP COLUMN preferred_locale;

   -- Clear translation cache (optional)
   TRUNCATE TABLE translation_cache;
   ```

---

## Common Issues & Troubleshooting

### Issue: Language toggle not saving preference
**Symptom**: User selects Urdu, but preference reverts to English on refresh

**Solution**:
1. Check Better Auth session is active: `console.log(session.user.id)`
2. Verify API call succeeds: `PUT /api/user/preferences` returns 200
3. Check database: `SELECT preferred_locale FROM users WHERE id = 'user_123';`

---

### Issue: Urdu content not displaying
**Symptom**: User in Urdu mode sees English content

**Solution**:
1. Check Qdrant payload: `content_ur` field present and non-null?
2. Verify `translation_status` != `'pending'`
3. Check Accept-Language header sent to API: `request.headers.get("accept-language")`

---

### Issue: RTL layout breaks code blocks
**Symptom**: Code appears reversed or right-aligned

**Solution**:
1. Verify CSS override applied:
   ```css
   [dir="rtl"] pre, [dir="rtl"] code {
     direction: ltr !important;
     text-align: left !important;
   }
   ```
2. Check Docusaurus theme customization not overriding

---

### Issue: Chatbot responses in wrong language
**Symptom**: Urdu query returns English response

**Solution**:
1. Check language detection: `detect_language("PID کیا ہے؟")` → Should return `"ur"`
2. Verify OpenAI prompt includes: `"Respond in Urdu"`
3. Check Accept-Language header: `request.headers.get("accept-language")`

---

## Success Criteria Checklist

Before marking feature complete, verify:

- [ ] **SC-001**: Language toggle < 2s
- [ ] **SC-002**: 95% UI elements correct in Urdu
- [ ] **SC-003**: Content retrieval < 200ms
- [ ] **SC-004**: 90%+ chatbot terminology accuracy
- [ ] **SC-005**: 80% cache hit rate
- [ ] **SC-006**: 100% preference persistence
- [ ] **SC-007**: 85%+ user satisfaction (survey)
- [ ] **SC-008**: 100% graceful fallback
- [ ] **SC-009**: 95% RTL browser consistency
- [ ] **SC-010**: 90%+ mixed-query accuracy

---

## Resources

- **Specification**: `specs/002-i18n/spec.md`
- **Research**: `specs/002-i18n/research.md`
- **Data Model**: `specs/002-i18n/data-model.md`
- **API Contracts**: `specs/002-i18n/contracts/api-spec.yaml`
- **Docusaurus i18n Docs**: https://docusaurus.io/docs/i18n/introduction
- **Better Auth Docs**: Use `better-auth` MCP server
- **OpenAI Translation**: https://platform.openai.com/docs/guides/gpt-best-practices

---

**Last Updated**: 2025-12-17
**Status**: ✅ Ready for Implementation
