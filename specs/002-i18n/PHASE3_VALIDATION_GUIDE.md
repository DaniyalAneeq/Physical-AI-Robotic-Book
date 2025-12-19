# Phase 3 Validation Guide - User Story 1: UI Localization

**Date**: 2025-12-17
**Phase**: 3 - User Story 1 Implementation (T041-T046)
**Status**: Ready for Validation Testing

---

## Overview

This guide provides step-by-step instructions for validating the bilingual UI (English/Urdu) with RTL support and auth-gated language toggle.

**Implemented Features:**
- ✅ Docusaurus i18n configuration with English and Urdu locales
- ✅ RTL layout for Urdu with LTR preservation for technical content
- ✅ Language toggle component (auth-gated)
- ✅ User language preference persistence (PostgreSQL)
- ✅ Comprehensive UI translations (200+ strings)

---

## Prerequisites

Before starting validation, ensure:

1. **Database Migrations Applied**:
   ```bash
   # Auth backend migration
   cd rag-chatbot/auth_backend
   alembic upgrade head

   # RAG backend migration
   cd ../backend
   alembic upgrade head
   ```

2. **Development Server Running**:
   ```bash
   # Terminal 1: Backend
   cd rag-chatbot/backend
   uvicorn app.main:main --reload --host 0.0.0.0 --port 8000

   # Terminal 2: Frontend
   cd AIdd-book
   npm run start
   ```

3. **Test User Account Created**:
   - Email: `test@example.com`
   - Password: `TestPassword123!`
   - OR use existing authenticated account

---

## Validation Tasks

### ✅ T041: Login → Language Toggle Visible

**Objective**: Verify language toggle appears only when user is authenticated

**Steps**:
1. Open browser to `http://localhost:3000`
2. **Before Login**:
   - Navigate to homepage
   - Check navbar (top-right corner)
   - **Expected**: Language toggle dropdown is **NOT visible**

3. **After Login**:
   - Click "Log In" button in navbar
   - Enter credentials and submit
   - **Expected**: Redirected to homepage
   - Check navbar (top-right corner)
   - **Expected**: Language toggle dropdown **IS visible** between color mode toggle and user menu

**Pass Criteria**:
- [ ] Language toggle hidden when logged out
- [ ] Language toggle appears when logged in
- [ ] Toggle positioned correctly in navbar (right side, before user menu)

**Screenshot Locations** (optional):
- `specs/002-i18n/validation/T041-logged-out.png`
- `specs/002-i18n/validation/T041-logged-in.png`

---

### ✅ T042: Select "اردو" → UI Switches to RTL, Navigation in Urdu

**Objective**: Verify complete UI language switch to Urdu with RTL layout

**Steps**:
1. Ensure you are logged in
2. Click language toggle dropdown in navbar
3. Select **"اردو"** (Urdu)
4. **Expected**: Page reloads with URL path change (`/ur/...`)
5. **Verify UI Elements** (check all):

   **Navbar**:
   - [ ] "صفحات" (Pages) instead of "Pages"
   - [ ] "بلاگ" (Blog) instead of "Blog"
   - [ ] Login button becomes "لاگ ان کریں"
   - [ ] Navbar layout flows right-to-left

   **Sidebar**:
   - [ ] Module titles in Urdu (e.g., "ماڈیول 1: ROS 2 بنیادیات")
   - [ ] Sidebar positioned on RIGHT side of screen (RTL layout)
   - [ ] Collapse/expand icons flipped correctly

   **Footer**:
   - [ ] Footer text in Urdu
   - [ ] Footer links aligned to right

   **Breadcrumbs**:
   - [ ] Breadcrumb separators show "◀" instead of "►"
   - [ ] Breadcrumb text flows RTL

   **Search**:
   - [ ] Search placeholder text in Urdu
   - [ ] Search input still accepts LTR queries

   **Pagination**:
   - [ ] "پچھلا" (Previous) on right
   - [ ] "اگلا" (Next) on left
   - [ ] Arrow directions flipped

**Pass Criteria**:
- [ ] All UI elements switch to Urdu
- [ ] RTL layout applied consistently
- [ ] No broken UI or layout issues
- [ ] Urdu text renders correctly (no tofu/boxes)

**Screenshot**: `specs/002-i18n/validation/T042-urdu-ui.png`

---

### ✅ T043: Refresh Page → Urdu Locale Persists

**Objective**: Verify language preference persists across page reloads

**Steps**:
1. Ensure you are on Urdu locale (`/ur/...` in URL)
2. **Refresh browser** (F5 or Ctrl+R)
3. **Expected**: Page reloads with Urdu UI still active
4. **Navigate to different page** (e.g., click a module in sidebar)
5. **Expected**: New page loads with Urdu UI
6. **Hard refresh** (Ctrl+Shift+R or Cmd+Shift+R)
7. **Expected**: Urdu UI still persists

**Backend Verification**:
1. Open browser DevTools → Network tab
2. Refresh page
3. **Expected**: No API call to `/api/user/preferences` on initial load
4. Language preference is determined by:
   - URL path (`/ur/`)
   - User's stored preference (PostgreSQL `users.preferred_locale`)

**Database Check** (optional):
```sql
-- Connect to Neon database
SELECT id, email, preferred_locale
FROM users
WHERE email = 'test@example.com';
```
**Expected**: `preferred_locale` column shows `'ur'`

**Pass Criteria**:
- [ ] Urdu UI persists after page refresh
- [ ] Urdu UI persists across navigation
- [ ] URL path maintains `/ur/` prefix
- [ ] Database stores `preferred_locale = 'ur'`

---

### ✅ T044: Code Examples Remain LTR Within RTL Layout

**Objective**: Verify technical content (code, diagrams, tables) stays LTR within RTL layout

**Steps**:
1. Ensure you are on Urdu locale
2. Navigate to a page with **code examples** (e.g., Module 1, Chapter 1)
3. **Verify Code Blocks**:
   - [ ] Python/C++ code is **left-to-right** (LTR)
   - [ ] Code is **left-aligned** (not right-aligned)
   - [ ] Line numbers (if present) are on **left side**
   - [ ] Syntax highlighting works correctly
   - [ ] Code scrolls left-to-right

4. **Verify Inline Code**:
   - [ ] Inline `code snippets` within Urdu text are LTR
   - [ ] Code snippets don't break RTL text flow

5. **Verify Tables** (if present):
   - [ ] Technical tables are left-aligned
   - [ ] Table headers are LTR
   - [ ] Table data flows LTR

6. **Verify Diagrams** (if present):
   - [ ] Mermaid diagrams are LTR
   - [ ] SVG diagrams maintain original orientation
   - [ ] Diagram captions (if present) are RTL

7. **Verify Math Equations** (if present):
   - [ ] LaTeX/KaTeX equations are LTR
   - [ ] Equations align left

**Example Test Case**:
```python
# This code block should be LEFT-ALIGNED and LTR
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

**RTL Text Above/Below Code**:
```
یہ ایک مثال ہے (This is an example)

[CODE BLOCK - LTR, left-aligned]

مزید تفصیلات کے لیے دیکھیں (See for more details)
```

**Pass Criteria**:
- [ ] All code blocks are LTR and left-aligned
- [ ] Inline code within RTL text is LTR
- [ ] Tables remain LTR
- [ ] Diagrams maintain original orientation
- [ ] No visual glitches or text overlap

**Screenshot**: `specs/002-i18n/validation/T044-code-ltr.png`

---

### ✅ T045: Logout → Language Toggle Hidden

**Objective**: Verify language toggle disappears when user logs out

**Steps**:
1. Ensure you are logged in with Urdu UI active
2. Click **User Menu** (profile icon in navbar)
3. Click **"لاگ آؤٹ"** (Logout)
4. **Expected**: Redirected to homepage or login page
5. Check navbar (top-right corner)
6. **Expected**: Language toggle dropdown is **NOT visible**

**Additional Checks**:
- [ ] After logout, UI remains in Urdu (URL still `/ur/...`)
- [ ] Auth buttons ("لاگ ان کریں", "سائن اپ کریں") are visible
- [ ] No errors in browser console

**Re-login Test**:
1. Click "لاگ ان کریں" (Login)
2. Enter credentials and submit
3. **Expected**: Language toggle reappears
4. **Expected**: UI switches to user's preferred locale (`ur`)

**Pass Criteria**:
- [ ] Language toggle hidden after logout
- [ ] No layout shifts or broken UI
- [ ] Urdu UI persists (URL-based)
- [ ] Language toggle reappears after re-login
- [ ] User's preferred locale is restored after login

---

### ✅ T046: Browser Compatibility Test

**Objective**: Verify UI works across major browsers

**Browsers to Test**:
1. **Google Chrome** (latest stable)
2. **Mozilla Firefox** (latest stable)
3. **Safari** (latest stable, macOS only)
4. **Microsoft Edge** (latest stable)

**Test Matrix**:

| Browser | Login | Toggle Visible | Switch to Urdu | RTL Layout | Code LTR | Persist | Logout |
|---------|-------|----------------|----------------|------------|----------|---------|--------|
| Chrome  | ☐     | ☐              | ☐              | ☐          | ☐        | ☐       | ☐      |
| Firefox | ☐     | ☐              | ☐              | ☐          | ☐        | ☐       | ☐      |
| Safari  | ☐     | ☐              | ☐              | ☐          | ☐        | ☐       | ☐      |
| Edge    | ☐     | ☐              | ☐              | ☐          | ☐        | ☐       | ☐      |

**For Each Browser**:
1. Run T041-T045 in sequence
2. Check for:
   - [ ] Urdu fonts render correctly
   - [ ] RTL layout works as expected
   - [ ] Language toggle functions properly
   - [ ] Code blocks remain LTR
   - [ ] No console errors
   - [ ] Responsive design works (mobile view)

**Mobile Testing** (optional):
- [ ] Test on mobile Chrome (Android)
- [ ] Test on mobile Safari (iOS)
- [ ] Verify language toggle in mobile sidebar
- [ ] Check RTL layout on small screens

**Pass Criteria**:
- [ ] All browsers pass T041-T045 tests
- [ ] No browser-specific visual bugs
- [ ] Urdu fonts render consistently
- [ ] Performance is acceptable (no lag)

**Known Issues** (document any):
- Safari: [describe issue if any]
- Firefox: [describe issue if any]
- Edge: [describe issue if any]

---

## Common Issues & Troubleshooting

### Issue 1: Language Toggle Not Visible After Login
**Symptoms**: Toggle doesn't appear even when logged in

**Debugging Steps**:
1. Open browser DevTools → Console
2. Check for errors related to `useAuth` hook
3. Verify `AuthProvider` is wrapping the app:
   ```tsx
   // AIdd-book/src/theme/Root.tsx
   <AuthProvider>
     {children}
   </AuthProvider>
   ```
4. Check if user object is populated:
   ```javascript
   // In browser console
   localStorage.getItem('auth_user')
   ```

**Fix**: Ensure `AuthProvider` is properly configured and session cookie is set

---

### Issue 2: UI Not Switching to Urdu
**Symptoms**: Clicking language toggle doesn't change UI language

**Debugging Steps**:
1. Check URL after clicking toggle - should show `/ur/...`
2. Check browser DevTools → Network tab
3. Verify API call to `/api/user/preferences` succeeds (200 status)
4. Check browser console for errors

**Fix**:
- Ensure backend API is running
- Verify CORS configuration allows credentials
- Check `i18n/ur/code.json` exists and is valid JSON

---

### Issue 3: Code Blocks Show RTL
**Symptoms**: Code blocks are right-aligned or RTL

**Debugging Steps**:
1. Open browser DevTools → Elements
2. Inspect code block element
3. Check computed styles for `direction` property
4. Verify `rtl.css` is loaded:
   ```css
   [dir="rtl"] pre,
   [dir="rtl"] code {
     direction: ltr !important;
     text-align: left !important;
   }
   ```

**Fix**: Ensure `rtl.css` is imported in `custom.css` and has proper CSS specificity

---

### Issue 4: Locale Not Persisting After Refresh
**Symptoms**: UI switches back to English after page reload

**Debugging Steps**:
1. Check URL path - should maintain `/ur/` prefix
2. Verify database:
   ```sql
   SELECT preferred_locale FROM users WHERE email = 'test@example.com';
   ```
3. Check browser DevTools → Application → Cookies
4. Verify session cookie is set

**Fix**:
- Ensure backend API properly saves `preferred_locale`
- Check database migration was applied
- Verify session cookie has correct domain/path

---

## Test Report Template

After completing all validation tasks, fill out this report:

```markdown
# Phase 3 Validation Test Report

**Date**: [YYYY-MM-DD]
**Tester**: [Your Name]
**Environment**:
- OS: [Windows/Mac/Linux]
- Node Version: [e.g., 20.11.0]
- Python Version: [e.g., 3.11.7]
- Browser: [Chrome/Firefox/Safari/Edge + version]

## Test Results Summary

| Task | Status | Notes |
|------|--------|-------|
| T041 | ✅/❌ | [Any issues?] |
| T042 | ✅/❌ | [Any issues?] |
| T043 | ✅/❌ | [Any issues?] |
| T044 | ✅/❌ | [Any issues?] |
| T045 | ✅/❌ | [Any issues?] |
| T046 | ✅/❌ | [Any issues?] |

## Pass/Fail Decision

**Overall Status**: ✅ PASS / ❌ FAIL

**Critical Issues** (blockers):
- [List any issues that prevent release]

**Non-Critical Issues** (can be fixed post-MVP):
- [List minor issues]

## Recommendations

- [Proceed to Phase 4]
- [Fix critical issues before proceeding]
- [Additional testing needed in X area]

## Screenshots

Attach screenshots to `specs/002-i18n/validation/`:
- T041-logged-out.png
- T041-logged-in.png
- T042-urdu-ui.png
- T044-code-ltr.png

**Signed Off By**: [Your Name]
```

---

## Next Steps

After all tests pass:

1. ✅ Mark tasks T041-T046 as complete in `tasks.md`
2. ✅ Update todo list to mark Phase 3 as fully complete
3. ✅ Create Prompt History Record (PHR) for Phase 3 validation
4. ✅ Proceed to **Phase 4: User Story 2 - Accessing Pre-translated Book Content** (T047-T066)

---

## References

- **Spec**: `specs/002-i18n/spec.md`
- **Tasks**: `specs/002-i18n/tasks.md`
- **Docusaurus i18n**: https://docusaurus.io/docs/i18n/introduction
- **RTL Support**: https://docusaurus.io/docs/i18n/tutorial#translate-your-site
- **Better Auth Docs**: (via MCP server)

---

**End of Phase 3 Validation Guide**
