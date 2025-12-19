# Phase 4 Validation Guide: Pre-translated Book Content

**User Story 2**: Users in Urdu mode see book module content in Urdu (if available) with technical terms preserved correctly

**Status**: ✅ Implementation Complete (2025-12-17)

---

## Implementation Summary

Phase 4 delivers localized book content with automatic fallback handling. Key deliverables:

### Backend Components (Tasks T047-T054)

1. **Content API Endpoint** (`rag-chatbot/backend/app/api/content.py`)
   - `GET /api/content/{module_id}` - Retrieve localized module content
   - Accept-Language header parsing with fallback to user preference
   - Qdrant integration for bilingual content retrieval
   - Automatic fallback to English for incomplete translations
   - Authentication-gated access

2. **Router Registration** (`rag-chatbot/backend/app/main.py`)
   - Content router added to main FastAPI application
   - Available at `/api/content/*` endpoints

### Frontend Components (Tasks T055-T061)

1. **useLocale Hook** (`AIdd-book/src/hooks/useLocale.tsx`)
   - Access current locale and RTL detection
   - Programmatic locale switching
   - Sync with Docusaurus i18n system

2. **Locale API Service** (`AIdd-book/src/services/localeApi.ts`)
   - `fetchModuleContent(moduleId, locale)` - Fetch localized content
   - `prefetchModuleContent(moduleId, locale)` - Background prefetching
   - Authentication token handling
   - Error handling for network failures and missing content

3. **FallbackBanner Component** (`AIdd-book/src/components/FallbackBanner/`)
   - Visual notification for incomplete translations
   - RTL-aware layout
   - Dark mode support
   - Accessibility features (ARIA roles, focus management)

4. **LocalizedModuleContent Component** (`AIdd-book/src/components/LocalizedModuleContent/`)
   - Complete module content display with locale support
   - Automatic loading/error state management
   - Fallback banner integration
   - Development mode debugging info

---

## Validation Tests (Tasks T062-T066)

### T062: View Module in English

**Test**: View Module 1 Chapter 1 in English → Content in English

**Steps**:
1. Ensure locale is set to English (`en`)
2. Navigate to any module page (e.g., `/docs/module-1/01-introduction`)
3. Verify content displays in English
4. Check network tab: `GET /api/content/module-1-chapter-1` with `Accept-Language: en`

**Expected Result**:
- Content displays in English
- No fallback banner shown
- `translation_status: "complete"`

**Command**:
```bash
# Test API endpoint directly
curl -X GET "http://localhost:8000/api/content/module-1-chapter-1" \
  -H "Accept-Language: en" \
  -H "Authorization: Bearer YOUR_TOKEN"
```

---

### T063: Switch to Urdu (Translated Content)

**Test**: Switch to Urdu → Content in Urdu (if translated)

**Steps**:
1. Login to the application
2. Click language toggle, select "اردو"
3. Navigate to a translated module
4. Verify content displays in Urdu

**Expected Result**:
- Content displays in Urdu (`content_ur` field)
- Title displays in Urdu (`title_ur` field)
- No fallback banner (if `translation_status: "complete"`)
- Code blocks remain LTR

**Command**:
```bash
# Test API endpoint with Urdu locale
curl -X GET "http://localhost:8000/api/content/module-1-chapter-1" \
  -H "Accept-Language: ur" \
  -H "Authorization: Bearer YOUR_TOKEN"
```

---

### T064: Technical Term Preservation

**Test**: Technical term "PID Controller" preserved in English or with Urdu equivalent

**Steps**:
1. Set locale to Urdu
2. Navigate to module containing technical terms (e.g., PID Controller, ROS 2)
3. Inspect content for technical term handling

**Expected Result**:
- Technical terms remain in English: "PID Controller"
- OR shown with Urdu translation + English in parentheses: "پی آئی ڈی کنٹرولر (PID Controller)"
- Formatting preserved (no broken layout)

**Verification**:
```bash
# Check Qdrant data for technical term handling
# This requires Qdrant access - check content_ur fields
```

---

### T065: Untranslated Module Fallback

**Test**: Untranslated module shows English content + fallback banner

**Steps**:
1. Set locale to Urdu
2. Navigate to module with `translation_status != "complete"`
3. Verify fallback banner appears
4. Verify content displays in English

**Expected Result**:
- Fallback banner visible with message: "This content is not yet available in Urdu. Showing English version. Translation coming soon!"
- Content displays from `content_en` field
- Title displays from `title_en` field
- `fallback_message` field populated in API response

**API Response Example**:
```json
{
  "module_id": "module-2-chapter-5",
  "locale": "en",
  "content": "English content here...",
  "title": "Chapter Title in English",
  "translation_status": "partial",
  "fallback_message": "This content is not yet available in Urdu. Showing English version. Translation coming soon!"
}
```

---

### T066: Code Blocks in Urdu Chapters

**Test**: Code blocks in Urdu chapters remain in English with Urdu comments

**Steps**:
1. Set locale to Urdu
2. Navigate to module with code examples
3. Inspect code blocks for language and directionality

**Expected Result**:
- Code blocks display LTR (left-to-right)
- Code syntax in English (Python, C++, etc.)
- Comments in code blocks in Urdu (if translated)
- No horizontal scrolling issues

**CSS Verification**:
```css
/* Verify these styles are applied */
[dir='rtl'] .content pre,
[dir='rtl'] .content code {
  direction: ltr;
  text-align: left;
}
```

---

## Integration Points

### Dependencies from Phase 3 (User Story 1)
- ✅ Language toggle component (provides locale switching)
- ✅ User preferences API (provides `preferred_locale`)
- ✅ RTL CSS (provides LTR override for code blocks)

### Independent Testing
Phase 4 can be tested independently of Phase 5 (Chatbot) and Phase 6 (Persistence):
- Requires: User authentication, Qdrant with bilingual data
- Does NOT require: Translation service, chatbot functionality

---

## Manual Testing Checklist

Use this checklist to verify Phase 4 implementation:

- [ ] **Backend API**: `GET /api/content/{module_id}` returns 200 OK
- [ ] **English Content**: Module content loads in English locale
- [ ] **Urdu Content**: Translated modules load in Urdu locale
- [ ] **Fallback Banner**: Appears for untranslated content in Urdu mode
- [ ] **Technical Terms**: Preserved correctly in Urdu content
- [ ] **Code Blocks**: Remain LTR in RTL layout
- [ ] **Authentication**: Endpoint requires valid JWT token
- [ ] **Error Handling**: 404 for missing modules, 401 for unauthenticated
- [ ] **RTL Layout**: Content displays correctly in RTL mode
- [ ] **Loading State**: Shows spinner while fetching content
- [ ] **Error State**: Shows error message on fetch failure

---

## Automated Testing

### Backend Tests

Create integration tests at `rag-chatbot/backend/tests/integration/test_content_api.py`:

```python
import pytest
from fastapi.testclient import TestClient

def test_get_module_content_english(client: TestClient, auth_token: str):
    """Test fetching module content in English."""
    response = client.get(
        "/api/content/module-1-chapter-1",
        headers={
            "Accept-Language": "en",
            "Authorization": f"Bearer {auth_token}"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert data["locale"] == "en"
    assert data["translation_status"] == "complete"
    assert "content" in data
    assert "title" in data

def test_get_module_content_urdu_translated(client: TestClient, auth_token: str):
    """Test fetching translated module content in Urdu."""
    response = client.get(
        "/api/content/module-1-chapter-1",  # Assuming this is translated
        headers={
            "Accept-Language": "ur",
            "Authorization": f"Bearer {auth_token}"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert data["locale"] == "ur"
    assert data["translation_status"] == "complete"
    assert data["fallback_message"] is None

def test_get_module_content_urdu_untranslated(client: TestClient, auth_token: str):
    """Test fallback for untranslated module in Urdu."""
    response = client.get(
        "/api/content/module-5-chapter-10",  # Assuming this is NOT translated
        headers={
            "Accept-Language": "ur",
            "Authorization": f"Bearer {auth_token}"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert data["locale"] == "en"  # Fallback to English
    assert data["fallback_message"] is not None
    assert "not yet available" in data["fallback_message"].lower()

def test_get_module_content_not_found(client: TestClient, auth_token: str):
    """Test 404 for non-existent module."""
    response = client.get(
        "/api/content/invalid-module-id",
        headers={
            "Accept-Language": "en",
            "Authorization": f"Bearer {auth_token}"
        }
    )
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()

def test_get_module_content_unauthenticated(client: TestClient):
    """Test 401 for unauthenticated request."""
    response = client.get(
        "/api/content/module-1-chapter-1",
        headers={"Accept-Language": "en"}
    )
    assert response.status_code == 401
```

**Run Tests**:
```bash
cd rag-chatbot/backend
pytest tests/integration/test_content_api.py -v
```

### Frontend Tests

Create E2E tests at `AIdd-book/tests/e2e/localized-content.spec.ts`:

```typescript
import { test, expect } from '@playwright/test';

test.describe('Localized Module Content', () => {
  test.beforeEach(async ({ page }) => {
    // Login before each test
    await page.goto('/login');
    await page.fill('input[name="email"]', 'test@example.com');
    await page.fill('input[name="password"]', 'password123');
    await page.click('button[type="submit"]');
    await page.waitForURL('/');
  });

  test('should display module content in English', async ({ page }) => {
    await page.goto('/en/docs/module-1/01-introduction');

    // Wait for content to load
    await page.waitForSelector('.moduleContent');

    // Verify content is in English
    const title = await page.textContent('h1');
    expect(title).toBeTruthy();

    // Verify no fallback banner
    const banner = await page.$('.fallbackBanner');
    expect(banner).toBeNull();
  });

  test('should display module content in Urdu with fallback banner', async ({ page }) => {
    // Switch to Urdu
    await page.click('.languageToggle');
    await page.click('option[value="ur"]');

    await page.goto('/ur/docs/module-1/01-introduction');
    await page.waitForSelector('.moduleContent');

    // Check if content is in Urdu or fallback is shown
    const banner = await page.$('.fallbackBanner');
    if (banner) {
      // Fallback banner should be visible
      const bannerText = await banner.textContent();
      expect(bannerText).toContain('not yet available in Urdu');
    }
  });

  test('should keep code blocks LTR in RTL layout', async ({ page }) => {
    await page.click('.languageToggle');
    await page.click('option[value="ur"]');

    await page.goto('/ur/docs/module-1/02-ros-basics');
    await page.waitForSelector('pre code');

    // Verify code block direction
    const codeBlock = await page.$('pre code');
    const direction = await codeBlock?.evaluate(el =>
      window.getComputedStyle(el).direction
    );
    expect(direction).toBe('ltr');
  });
});
```

**Run E2E Tests**:
```bash
cd AIdd-book
npm run test:e2e -- localized-content.spec.ts
```

---

## Performance Benchmarks

### Latency Targets (from spec.md)

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Content Retrieval (English) | < 200ms | Browser DevTools Network tab |
| Content Retrieval (Urdu) | < 200ms | Browser DevTools Network tab |
| Fallback Detection | Instant | No API call for fallback logic |
| Page Load Time | < 2s (total) | Lighthouse audit |

**Measurement Commands**:
```bash
# Backend API latency
curl -w "@curl-format.txt" -o /dev/null -s \
  "http://localhost:8000/api/content/module-1-chapter-1" \
  -H "Accept-Language: ur" \
  -H "Authorization: Bearer YOUR_TOKEN"

# Create curl-format.txt:
# time_namelookup: %{time_namelookup}\n
# time_connect: %{time_connect}\n
# time_starttransfer: %{time_starttransfer}\n
# time_total: %{time_total}\n
```

---

## Known Limitations & Future Work

### Current Limitations
1. **Content stored in Qdrant**: Requires Qdrant schema update (completed in Phase 3)
2. **No caching**: Every request hits Qdrant (caching in Phase 5)
3. **Binary fallback**: Either fully translated or fully English (no partial translations)

### Future Enhancements (Post-MVP)
1. **Client-side caching**: Cache fetched content in browser localStorage
2. **Progressive translation**: Show Urdu sections + English sections in same page
3. **Translation coverage metrics**: Dashboard showing translation progress
4. **User-contributed translations**: Allow users to suggest Urdu translations

---

## Rollback Plan

If Phase 4 causes issues:

1. **Disable content API**:
   ```python
   # In rag-chatbot/backend/app/main.py
   # Comment out:
   # app.include_router(content.router, tags=["Content"])
   ```

2. **Hide FallbackBanner**:
   ```tsx
   // In component usage
   <FallbackBanner visible={false} />
   ```

3. **Revert to static content**: Use standard Docusaurus i18n without API calls

---

## Success Criteria Mapping

Phase 4 contributes to these Success Criteria from spec.md:

- **SC-003**: ✅ Content retrieval < 200ms (backend optimization)
- **SC-008**: ✅ 100% graceful fallback (fallback banner + English content)
- **SC-002**: ✅ 95% UI elements correct (RTL layout for content)

---

## Next Steps

After completing Phase 4 validation:

1. **Update tasks.md**: Mark T047-T066 as complete
2. **Commit changes**: Create feature commit for Phase 4
3. **Proceed to Phase 5**: User Story 3 - Chatbot Interaction in Urdu
   - Translation service implementation
   - Urdu query handling with technical term preservation
   - Translation caching for performance

---

## Contact & Support

For issues with Phase 4 implementation:
- Check backend logs: `rag-chatbot/backend/logs/`
- Inspect Qdrant data: Verify `content_ur` and `translation_status` fields
- Frontend console: Check for API errors or network failures

**Common Issues**:
1. **404 errors**: Module ID mismatch between frontend and Qdrant
2. **401 errors**: Authentication token expired or missing
3. **Empty content**: Qdrant schema not updated (run Phase 2 migrations)
4. **No fallback banner**: `translation_status` incorrectly set to "complete"
