# Phase 4 Quick Start Guide

**For**: Developers implementing localized content in book modules
**Time**: 5 minutes
**Prerequisites**: Phase 3 complete (language toggle working)

---

## TL;DR - 3 Ways to Use Phase 4

### Option 1: Drop-in Component (Easiest)

```tsx
// In your MDX file (e.g., docs/module-1/chapter-1.mdx)
import LocalizedModuleContent from '@site/src/components/LocalizedModuleContent';

<LocalizedModuleContent moduleId="module-1-chapter-1" />
```

**Pros**: Zero configuration, automatic locale handling, fallback banner
**Cons**: Less control over loading/error states

---

### Option 2: Custom Hook + API (Most Flexible)

```tsx
import { useEffect, useState } from 'react';
import useLocale from '@site/src/hooks/useLocale';
import { fetchModuleContent } from '@site/src/services/localeApi';

function MyModulePage() {
  const { locale } = useLocale();
  const [content, setContent] = useState(null);

  useEffect(() => {
    fetchModuleContent('module-1-chapter-1', locale)
      .then(setContent)
      .catch(console.error);
  }, [locale]);

  if (!content) return <div>Loading...</div>;

  return <div dangerouslySetInnerHTML={{ __html: content.content }} />;
}
```

**Pros**: Full control, custom UI, advanced features
**Cons**: More code to write

---

### Option 3: Static i18n (No API)

```mdx
// docs/module-1/chapter-1.mdx (English)
# Introduction to ROS 2
Content in English...

// i18n/ur/docusaurus-plugin-content-docs/module-1/chapter-1.mdx (Urdu)
# ROS 2 کا تعارف
اردو میں مواد...
```

**Pros**: No API calls, Docusaurus native
**Cons**: Duplicate files, harder to update translations

---

## API Reference (Quick)

### Backend: Get Module Content

```bash
GET /api/content/{module_id}
Headers:
  Accept-Language: en|ur
  Authorization: Bearer <JWT>

Response:
{
  "module_id": "module-1-chapter-1",
  "locale": "ur",
  "content": "<p>اردو میں مواد</p>",
  "title": "باب 1",
  "translation_status": "complete",
  "fallback_message": null
}
```

### Frontend: useLocale Hook

```tsx
const { locale, isRTL, setLocale } = useLocale();

// locale: 'en' | 'ur'
// isRTL: boolean (true for Urdu)
// setLocale: (locale: string) => void
```

### Frontend: Fetch Content

```tsx
import { fetchModuleContent } from '@site/src/services/localeApi';

const content = await fetchModuleContent('module-1-chapter-1', 'ur');
// Returns: { module_id, locale, content, title, translation_status, fallback_message }
```

### Frontend: Fallback Banner

```tsx
import FallbackBanner from '@site/src/components/FallbackBanner';

<FallbackBanner
  message="Translation coming soon!"
  visible={translationStatus !== 'complete'}
/>
```

---

## Common Patterns

### 1. Module Page with Fallback

```tsx
import LocalizedModuleContent from '@site/src/components/LocalizedModuleContent';

export default function Module1Chapter1() {
  return (
    <div>
      <h1>Chapter 1: Introduction</h1>
      <LocalizedModuleContent moduleId="module-1-chapter-1" />
    </div>
  );
}
```

### 2. Prefetch Next Chapter

```tsx
import { useEffect } from 'react';
import { prefetchModuleContent } from '@site/src/services/localeApi';
import useLocale from '@site/src/hooks/useLocale';

export default function ChapterPage({ currentId, nextId }) {
  const { locale } = useLocale();

  useEffect(() => {
    // Prefetch next chapter in background
    prefetchModuleContent(nextId, locale);
  }, [nextId, locale]);

  return <LocalizedModuleContent moduleId={currentId} />;
}
```

### 3. Custom Loading Spinner

```tsx
<LocalizedModuleContent
  moduleId="module-1-chapter-1"
  loadingComponent={
    <div className="custom-spinner">
      <img src="/spinner.gif" alt="Loading..." />
    </div>
  }
/>
```

### 4. Custom Error Handling

```tsx
<LocalizedModuleContent
  moduleId="module-1-chapter-1"
  errorComponent={(error) => (
    <div className="error-box">
      <h2>Oops!</h2>
      <p>{error.message}</p>
      <button onClick={() => window.location.reload()}>Retry</button>
    </div>
  )}
/>
```

---

## Styling Tips

### RTL-Aware Content

```css
/* Content automatically uses dir attribute */
.content {
  direction: inherit; /* Inherits from parent dir="rtl" or dir="ltr" */
}

/* Force LTR for code blocks in RTL layout */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}
```

### Dark Mode Support

```css
.myComponent {
  background-color: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
}

[data-theme='dark'] .myComponent {
  /* Dark mode styles automatically applied via CSS vars */
}
```

---

## Troubleshooting

### Issue: Content not loading

**Symptoms**: Spinner forever, no content
**Causes**:
1. Module ID mismatch (frontend vs Qdrant)
2. Authentication token missing/expired
3. Qdrant not running or schema not updated

**Fix**:
```bash
# Check backend logs
cd rag-chatbot/backend
tail -f logs/app.log

# Test API directly
curl -X GET "http://localhost:8000/api/content/module-1-chapter-1" \
  -H "Accept-Language: ur" \
  -H "Authorization: Bearer YOUR_TOKEN"

# Verify Qdrant schema (should have content_ur, title_ur fields)
# See PHASE4_VALIDATION_GUIDE.md for details
```

---

### Issue: Fallback banner not showing

**Symptoms**: No banner even when content is in English but locale is Urdu
**Causes**:
1. `translation_status` incorrectly set to "complete"
2. `fallback_message` not returned from API

**Fix**:
```tsx
// Force show banner for debugging
<FallbackBanner visible={true} message="Debug: Always visible" />

// Check API response
console.log('Translation status:', content.translation_status);
console.log('Fallback message:', content.fallback_message);
```

---

### Issue: Code blocks showing RTL

**Symptoms**: Code blocks reversed in Urdu mode
**Causes**: RTL CSS override not applied

**Fix**:
```css
/* Add to your CSS (should already be in rtl.css) */
[dir='rtl'] .content pre,
[dir='rtl'] .content code,
[dir='rtl'] pre code {
  direction: ltr !important;
  text-align: left !important;
}
```

---

### Issue: 401 Unauthorized

**Symptoms**: API returns 401, content not loaded
**Causes**: JWT token missing or invalid

**Fix**:
```tsx
// Check if token exists
const token = localStorage.getItem('auth_token');
console.log('Token:', token ? 'exists' : 'missing');

// Verify token not expired (decode JWT)
const payload = JSON.parse(atob(token.split('.')[1]));
console.log('Token expires:', new Date(payload.exp * 1000));
```

---

## Performance Tips

### 1. Prefetch Adjacent Chapters

```tsx
useEffect(() => {
  // Prefetch previous and next chapters
  prefetchModuleContent(prevChapterId, locale);
  prefetchModuleContent(nextChapterId, locale);
}, [prevChapterId, nextChapterId, locale]);
```

### 2. Memoize Content Rendering

```tsx
import { useMemo } from 'react';

const renderedContent = useMemo(
  () => ({ __html: content.content }),
  [content.content]
);

return <div dangerouslySetInnerHTML={renderedContent} />;
```

### 3. Lazy Load Components

```tsx
import { lazy, Suspense } from 'react';

const LocalizedModuleContent = lazy(() =>
  import('@site/src/components/LocalizedModuleContent')
);

export default function Page() {
  return (
    <Suspense fallback={<div>Loading component...</div>}>
      <LocalizedModuleContent moduleId="module-1-chapter-1" />
    </Suspense>
  );
}
```

---

## Testing Your Implementation

### Manual Test Checklist

1. [ ] Content loads in English
2. [ ] Content loads in Urdu (if translated)
3. [ ] Fallback banner appears for untranslated content
4. [ ] Code blocks remain LTR in RTL layout
5. [ ] Loading spinner shows during fetch
6. [ ] Error message shows on failure
7. [ ] Locale change triggers re-fetch

### Quick Test Script

```bash
# Start backend
cd rag-chatbot/backend
uvicorn app.main:app --reload

# Start frontend
cd AIdd-book
npm start

# Test in browser
# 1. Login
# 2. Navigate to module page
# 3. Switch language toggle to Urdu
# 4. Verify content changes or fallback banner appears
```

---

## Next Steps

1. **Implement in Your Module**: Choose Option 1, 2, or 3 above
2. **Test Locally**: Use manual test checklist
3. **Verify Fallback**: Test with untranslated module ID
4. **Performance Check**: Measure load time (< 200ms target)
5. **Deploy**: Merge to main after validation

---

## Need Help?

- **Validation Guide**: See `PHASE4_VALIDATION_GUIDE.md` for detailed tests
- **Full Documentation**: See `PHASE4_SUMMARY.md` for architecture details
- **API Errors**: Check backend logs at `rag-chatbot/backend/logs/`
- **Frontend Errors**: Open browser DevTools console

---

**Quick Start Complete!** You're now ready to implement localized content in your book modules.
