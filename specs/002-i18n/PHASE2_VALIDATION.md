# Phase 2: Foundational - Validation Checklist

**Status**: Code Complete - Ready for Migration Execution
**Date**: 2025-12-17
**Feature**: 002-i18n

---

## Phase 2 Summary

**Purpose**: Establish critical database layer and core infrastructure that blocks all user stories.

**Tasks Completed**: T001-T020 (code complete)
**Tasks Pending Execution**: T010 (migrations), T013 (Qdrant update)

---

## Files Created/Modified

### Database Migrations

✅ **Created**: `rag-chatbot/auth_backend/migrations/003_add_i18n_support.py`
- Adds `users.preferred_locale` column (VARCHAR(5), DEFAULT 'en')
- Adds CHECK constraint for supported locales ('en', 'ur')
- Adds index on preferred_locale
- Includes upgrade() and downgrade() functions

✅ **Created**: `rag-chatbot/backend/alembic/versions/002_add_i18n_support.py`
- Creates `translation_cache` table
- Adds indexes for query_hash, languages, last_accessed
- Adds CHECK constraints for supported languages
- Includes upgrade() and downgrade() functions

### Qdrant Schema Update

✅ **Created**: `rag-chatbot/backend/scripts/update_qdrant_schema.py`
- Non-destructive schema update script
- Adds fields: content_ur, title_ur, translation_status, technical_terms
- Includes dry-run mode (--apply flag to execute)
- Includes verification mode (--verify flag)

### Core Backend Infrastructure

✅ **Created**: `rag-chatbot/backend/app/utils/__init__.py`
✅ **Created**: `rag-chatbot/backend/app/utils/locale.py`
- `parse_accept_language()`: Parse HTTP Accept-Language header
- `get_user_locale()`: Resolve user's preferred locale (header → DB → default)
- `validate_locale()`: Validate locale is supported

✅ **Created**: `rag-chatbot/backend/app/services/translation_cache.py`
- `compute_query_hash()`: SHA-256 hash for cache keys
- `get_cached_translation()`: Retrieve cached translation with stat updates
- `cache_translation()`: Store new translation
- `get_cache_stats()`: Analytics (hit rate, average access count)

### Database Models

✅ **Modified**: `rag-chatbot/auth_backend/models/user.py`
- Added `preferred_locale: Mapped[str]` field
- Default: 'en', indexed

✅ **Created**: `rag-chatbot/backend/app/models/translation_cache.py`
- TranslationCache model with all required fields
- Includes __repr__ for debugging

✅ **Modified**: `rag-chatbot/backend/app/models/__init__.py`
- Added TranslationCache to exports

### Dependencies

✅ **Modified**: `rag-chatbot/backend/pyproject.toml`
- Added `langdetect==1.0.9`

### Project Structure

✅ **Created**: `AIdd-book/i18n/ur/` (directory)
✅ **Created**: `rag-chatbot/backend/app/utils/` (directory)
✅ **Verified**: `rag-chatbot/backend/app/services/` (already exists)

---

## Pre-Migration Validation Checklist

Before running T010 and T013, verify:

### Environment Variables

- [ ] `QDRANT_URL` set in `rag-chatbot/backend/.env`
- [ ] `QDRANT_API_KEY` set in `rag-chatbot/backend/.env`
- [ ] `QDRANT_COLLECTION_NAME` set (or defaults to 'book_content')
- [ ] Database connection strings valid in both backends

### Database Connectivity

- [ ] Auth backend can connect to PostgreSQL:
  ```bash
  cd rag-chatbot/auth_backend
  python -c "from database import get_db; next(get_db())"
  ```

- [ ] RAG backend can connect to PostgreSQL:
  ```bash
  cd rag-chatbot/backend
  python -c "from app.models.database import get_db; next(get_db())"
  ```

- [ ] Qdrant connectivity test:
  ```bash
  cd rag-chatbot/backend
  python -c "from qdrant_client import QdrantClient; import os; client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print(client.get_collections())"
  ```

### Backup Verification

- [ ] PostgreSQL backup created (or confirm Neon has automated backups)
- [ ] Qdrant snapshot exists (or confirm can restore if needed)

---

## Migration Execution Steps

### Step 1: Run Auth Backend Migration (T010 - Part 1)

```bash
cd rag-chatbot/auth_backend

# Run migration
python run_migrations.py

# Verify migration applied
python -c "from models.user import User; print(User.__table__.columns.keys())"
# Should include 'preferred_locale'
```

**Expected Output**:
- Migration 003_add_i18n_support applied successfully
- `users` table now has `preferred_locale` column

### Step 2: Run RAG Backend Migration (T010 - Part 2)

```bash
cd rag-chatbot/backend

# Run migration
alembic upgrade head

# Verify migration applied
alembic current
# Should show: 002 (head)
```

**Expected Output**:
- Migration 002_add_i18n_support applied successfully
- `translation_cache` table created

### Step 3: Run Qdrant Schema Update (T013)

```bash
cd rag-chatbot/backend

# Dry run first (verify what would change)
python scripts/update_qdrant_schema.py

# If dry run looks good, apply changes
python scripts/update_qdrant_schema.py --apply

# Verify schema update
python scripts/update_qdrant_schema.py --apply --verify
```

**Expected Output**:
- All existing points updated with new fields
- English content preserved (content_en unchanged)
- Sample point verification shows: content_ur, title_ur, translation_status, technical_terms

---

## Post-Migration Validation

### PostgreSQL Schema Validation

```sql
-- Auth backend database
SELECT column_name, data_type, column_default, is_nullable
FROM information_schema.columns
WHERE table_name = 'users' AND column_name = 'preferred_locale';
-- Expected: preferred_locale | character varying(5) | 'en' | NO

-- RAG backend database
SELECT table_name FROM information_schema.tables WHERE table_name = 'translation_cache';
-- Expected: translation_cache

SELECT COUNT(*) FROM translation_cache;
-- Expected: 0 (empty table)
```

### Qdrant Schema Validation

```python
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))

# Get sample point
points, _ = client.scroll(collection_name="book_content", limit=1, with_payload=True)
sample = points[0].payload

# Verify new fields present
required_fields = ['content_ur', 'title_ur', 'translation_status', 'technical_terms']
for field in required_fields:
    assert field in sample, f"Missing field: {field}"

# Verify English content unchanged
assert 'content_en' in sample
assert sample['content_en'] is not None
assert sample['translation_status'] == 'pending'

print("✓ Qdrant schema validation passed")
```

---

## Rollback Plan

If migration fails or issues discovered:

### Rollback Auth Backend Migration

```bash
cd rag-chatbot/auth_backend
# Rollback to previous migration
python run_migrations.py --downgrade
```

### Rollback RAG Backend Migration

```bash
cd rag-chatbot/backend
alembic downgrade -1
```

### Rollback Qdrant Schema

Qdrant schema update is non-destructive (only adds fields). To rollback:

```python
# Remove added fields from all points
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))

offset = None
while True:
    points, next_offset = client.scroll(
        collection_name="book_content",
        limit=100,
        offset=offset,
        with_payload=True
    )

    if not points:
        break

    for point in points:
        # Remove i18n fields
        payload = point.payload.copy()
        payload.pop('content_ur', None)
        payload.pop('title_ur', None)
        payload.pop('translation_status', None)
        payload.pop('technical_terms', None)

        client.upsert(
            collection_name="book_content",
            points=[PointStruct(id=point.id, vector=point.vector, payload=payload)]
        )

    if next_offset is None:
        break
    offset = next_offset
```

---

## Success Criteria

Phase 2 is complete when ALL of the following are true:

- [X] All migration files created (T005-T009)
- [X] Qdrant schema update script created (T011-T012)
- [X] Core utilities implemented (T014-T018)
- [X] Database models updated (T019-T020)
- [ ] Auth backend migration applied successfully (T010 - Part 1)
- [ ] RAG backend migration applied successfully (T010 - Part 2)
- [ ] Qdrant schema update applied successfully (T013)
- [ ] Post-migration validation passes
- [ ] No production errors or downtime

**Once complete**, user story implementation (Phase 3-6) can begin in parallel.

---

## Next Steps After Validation

1. **If all validations pass**: Proceed to Phase 3 (User Story 1 - UI Localization)
2. **If validation fails**: Debug issues, apply fixes, re-run validation
3. **If rollback needed**: Execute rollback plan, investigate root cause

---

**Status**: ⏸️ **Awaiting Migration Execution** (T010, T013)
**Blocker**: Requires database access and manual execution
**Risk Level**: Medium (non-destructive migrations, but affects production data)
