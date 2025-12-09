# Tasks: Modern Frontend Redesign

**Input**: Design documents from `/specs/001-frontend-redesign/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Visual testing via Lighthouse CLI and manual browser testing. No unit tests required for CSS-focused redesign.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project root**: `AIdd-book/` (Docusaurus project)
- **CSS**: `AIdd-book/src/css/`
- **Components**: `AIdd-book/src/components/`
- **Pages**: `AIdd-book/src/pages/`
- **Static assets**: `AIdd-book/static/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and design token foundation

- [x] T001 Create `AIdd-book/src/css/tokens.css` with all design tokens from contracts/design-tokens.css
- [x] T002 Create `AIdd-book/src/css/animations.css` with base animation keyframes and utilities
- [x] T003 Update `AIdd-book/src/css/custom.css` to import tokens.css and animations.css
- [x] T004 [P] Map design tokens to Infima variables in `AIdd-book/src/css/custom.css` (--ifm-* overrides)
- [x] T005 [P] Add dark theme token overrides in `AIdd-book/src/css/custom.css` using [data-theme='dark']
- [x] T006 Verify theme toggle switches colors correctly by testing in browser

**Checkpoint**: Design token foundation ready - all subsequent phases can use tokens

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core layout components that multiple user stories depend on

**CRITICAL**: These components MUST be complete before user story-specific work can begin

- [x] T007 Create `AIdd-book/src/components/SectionContainer/index.tsx` implementing SectionContainerProps interface
- [x] T008 Create `AIdd-book/src/components/SectionContainer/styles.module.css` with background variants (default, muted, primary, gradient)
- [x] T009 [P] Add responsive padding utilities to `AIdd-book/src/css/custom.css` for section layouts
- [x] T010 [P] Add max-width constraint utilities to `AIdd-book/src/css/custom.css`
- [x] T011 Add `prefers-reduced-motion` media query wrapper in `AIdd-book/src/css/animations.css`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - First Impression and Visual Appeal (Priority: P1)

**Goal**: Visitors perceive a professional, modern platform through cohesive visual design, typography, and whitespace

**Independent Test**: Load homepage on various devices, evaluate visual consistency and professional appearance

### Implementation for User Story 1

- [x] T012 [US1] Create `AIdd-book/src/components/HeroSection/index.tsx` implementing HeroSectionProps interface
- [x] T013 [US1] Create `AIdd-book/src/components/HeroSection/styles.module.css` with gradient background and animations
- [x] T014 [P] [US1] Add hero animations (fadeIn, slideUp) to `AIdd-book/src/css/animations.css`
- [x] T015 [US1] Update `AIdd-book/src/pages/index.tsx` to use new HeroSection component
- [x] T016 [US1] Remove old hero styles from `AIdd-book/src/pages/index.module.css` and replace with token-based styles
- [x] T017 [P] [US1] Update global typography in `AIdd-book/src/css/custom.css` (font sizes, weights, line heights)
- [x] T018 [P] [US1] Add visual hierarchy styles to `AIdd-book/src/css/custom.css` (headings, spacing, contrast)
- [x] T019 [US1] Redesign `AIdd-book/src/components/HomepageFeatures/styles.module.css` using design tokens
- [x] T020 [US1] Update `AIdd-book/src/components/HomepageFeatures/index.tsx` to use local images instead of picsum.photos
- [x] T021 [P] [US1] Add feature card images to `AIdd-book/static/img/` (ai-robotics.svg, hands-on.svg, research.svg)
- [x] T022 [US1] Redesign `AIdd-book/src/components/CallToAction/styles.module.css` using design tokens
- [x] T023 [US1] Update `AIdd-book/src/components/CallToAction/index.tsx` with proper button styling
- [x] T024 [US1] Verify color contrast meets WCAG AA (4.5:1 body text, 3:1 large text) using browser DevTools

**Checkpoint**: Homepage looks visually cohesive and professional - User Story 1 complete

---

## Phase 4: User Story 2 - Responsive Multi-Device Experience (Priority: P2)

**Goal**: Users on any device experience fully functional interface that adapts seamlessly to screen size

**Independent Test**: Test at breakpoints 320px, 768px, 1024px, 1440px - verify no horizontal scrolling, proper touch targets

### Implementation for User Story 2

- [x] T025 [US2] Add responsive breakpoint variables to `AIdd-book/src/css/custom.css` (mobile, tablet, laptop, desktop)
- [x] T026 [P] [US2] Add mobile styles to `AIdd-book/src/components/HeroSection/styles.module.css` (< 768px)
- [x] T027 [P] [US2] Add tablet styles to `AIdd-book/src/components/HeroSection/styles.module.css` (768-1023px)
- [x] T028 [US2] Add responsive grid to `AIdd-book/src/components/HomepageFeatures/styles.module.css` (1-col mobile, 2-col tablet, 3-col desktop)
- [x] T029 [US2] Ensure touch targets are minimum 44x44px in `AIdd-book/src/css/custom.css` (buttons, links)
- [x] T030 [P] [US2] Add responsive typography scaling to `AIdd-book/src/css/custom.css` (fluid font sizes)
- [x] T031 [US2] Add large screen max-width container (> 2560px content centered) in `AIdd-book/src/css/custom.css`
- [x] T032 [US2] Add responsive table styles to `AIdd-book/src/css/custom.css` (horizontal scroll on mobile)
- [x] T033 [US2] Test all pages at 320px, 768px, 1024px, 1440px, 2560px viewports

**Checkpoint**: Site works on all device sizes - User Story 2 complete

---

## Phase 5: User Story 3 - Smooth Navigation and Transitions (Priority: P2)

**Goal**: Users experience fluid, non-jarring transitions that enhance perceived quality

**Independent Test**: Navigate through site flows, time transitions (should complete within 300ms)

### Implementation for User Story 3

- [x] T034 [US3] Add navbar transition styles to `AIdd-book/src/css/custom.css` (scroll state changes)
- [x] T035 [P] [US3] Add link hover animations to `AIdd-book/src/css/custom.css` (150-200ms ease-out)
- [x] T036 [P] [US3] Add button state transitions to `AIdd-book/src/css/custom.css` (hover, focus, active)
- [x] T037 [US3] Add sidebar expand/collapse animation to `AIdd-book/src/css/custom.css` (doc sidebar)
- [x] T038 [P] [US3] Add card hover animations to `AIdd-book/src/components/HomepageFeatures/styles.module.css`
- [x] T039 [US3] Add mobile menu animation to `AIdd-book/src/css/custom.css` (hamburger menu open/close)
- [x] T040 [US3] Add focus ring transitions to `AIdd-book/src/css/custom.css` (visible focus indicators)
- [x] T041 [US3] Verify all animations complete within 300ms timing budget

**Checkpoint**: All interactions feel smooth and responsive - User Story 3 complete

---

## Phase 6: User Story 4 - Fast Performance and Load Times (Priority: P3)

**Goal**: Pages load quickly, Core Web Vitals targets met (LCP < 2.5s, CLS < 0.1, FID < 100ms)

**Independent Test**: Run Lighthouse audit, verify scores meet targets

### Implementation for User Story 4

- [x] T042 [US4] Optimize hero background image in `AIdd-book/static/img/` (WebP format, appropriate dimensions)
- [x] T043 [P] [US4] Replace external image URLs with optimized local images in `AIdd-book/src/components/HomepageFeatures/index.tsx`
- [x] T044 [US4] Add image dimension attributes to prevent CLS in `AIdd-book/src/components/HomepageFeatures/index.tsx`
- [x] T045 [P] [US4] Add font-display: swap to web fonts in `AIdd-book/src/css/custom.css`
- [x] T046 [US4] Add skeleton/placeholder styles for loading states in `AIdd-book/src/css/custom.css`
- [x] T047 [US4] Verify no layout shift on page load by testing with Lighthouse
- [x] T048 [US4] Run Lighthouse Performance audit and achieve 90+ score on mobile

**Checkpoint**: Performance targets met - User Story 4 complete

---

## Phase 7: User Story 5 - Accessible and Inclusive Design (Priority: P3)

**Goal**: Users with disabilities can fully access all site functionality

**Independent Test**: Run Lighthouse Accessibility audit (target 90+), test with keyboard only

### Implementation for User Story 5

- [x] T049 [US5] Add visible focus indicators to `AIdd-book/src/css/custom.css` (outline styles for all interactive elements)
- [x] T050 [P] [US5] Add skip-to-content link to `AIdd-book/src/css/custom.css` (visually hidden until focused)
- [x] T051 [US5] Verify semantic HTML in `AIdd-book/src/pages/index.tsx` (proper heading hierarchy, landmarks)
- [x] T052 [P] [US5] Add alt text to all images in `AIdd-book/src/components/HomepageFeatures/index.tsx`
- [x] T053 [US5] Add ARIA labels to icon-only buttons in components
- [x] T054 [US5] Verify color contrast in both light and dark themes using browser DevTools
- [x] T055 [US5] Test keyboard navigation flow (Tab order should be logical)
- [x] T056 [US5] Run Lighthouse Accessibility audit and achieve 90+ score

**Checkpoint**: Site meets WCAG AA standards - User Story 5 complete

---

## Phase 8: User Story 6 - Theme Consistency and Dark Mode (Priority: P3)

**Goal**: Users can view site in light or dark mode with both themes visually consistent

**Independent Test**: Toggle system theme preference, verify all pages render correctly in both modes

### Implementation for User Story 6

- [x] T057 [US6] Verify all colors use CSS custom properties in `AIdd-book/src/css/custom.css` (no hardcoded colors)
- [x] T058 [P] [US6] Add dark theme surface colors to `AIdd-book/src/components/HomepageFeatures/styles.module.css`
- [x] T059 [P] [US6] Add dark theme styles to `AIdd-book/src/components/CallToAction/styles.module.css`
- [x] T060 [US6] Add dark theme styles to `AIdd-book/src/components/HeroSection/styles.module.css`
- [x] T061 [US6] Verify dark mode image contrast (add filter or alternative images if needed)
- [x] T062 [US6] Test theme toggle functionality and verify smooth transition
- [x] T063 [US6] Verify brand colors are recognizable in both themes

**Checkpoint**: Both themes look polished and intentional - User Story 6 complete

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final quality assurance and cross-browser testing

- [x] T064 [P] Update `AIdd-book/docusaurus.config.ts` theme configuration (if needed)
- [x] T065 [P] Add documentation page styles to `AIdd-book/src/css/custom.css` (code blocks, tables, blockquotes)
- [x] T066 Update footer styles in `AIdd-book/src/css/custom.css` to match design system
- [x] T067 [P] Add navbar scroll behavior styles to `AIdd-book/src/css/custom.css`
- [x] T068 Cross-browser test in Chrome (latest)
- [x] T069 [P] Cross-browser test in Firefox (latest)
- [x] T070 [P] Cross-browser test in Safari (latest)
- [x] T071 [P] Cross-browser test in Edge (latest)
- [x] T072 Test on iOS Safari mobile browser
- [x] T073 Final Lighthouse audit (Performance 90+, Accessibility 90+)
- [x] T074 Clean up unused CSS and code
- [x] T075 Verify all responsive breakpoints work correctly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational phase - Visual foundation
- **User Story 2 (Phase 4)**: Depends on Phase 1-2 - Can parallel with US3-6
- **User Story 3 (Phase 5)**: Depends on Phase 1-2 - Can parallel with US2, US4-6
- **User Story 4 (Phase 6)**: Depends on Phase 1-3 (needs components to optimize)
- **User Story 5 (Phase 7)**: Depends on Phase 1-3 (needs components to make accessible)
- **User Story 6 (Phase 8)**: Depends on Phase 1-3 (needs components for theming)
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Foundation for all visual work - complete first
- **User Story 2 (P2)**: Can parallel with US3 after US1 complete
- **User Story 3 (P2)**: Can parallel with US2 after US1 complete
- **User Story 4 (P3)**: Depends on US1-3 (optimize existing components)
- **User Story 5 (P3)**: Depends on US1-3 (make existing components accessible)
- **User Story 6 (P3)**: Depends on US1-3 (theme existing components)

### Parallel Opportunities by Phase

**Phase 1 (Setup):**
- T004, T005 can run in parallel

**Phase 2 (Foundational):**
- T009, T010 can run in parallel

**Phase 3 (US1):**
- T014, T017, T018, T21 can run in parallel

**Phase 4 (US2):**
- T026, T027, T30 can run in parallel

**Phase 5 (US3):**
- T035, T036, T38 can run in parallel

**Phase 6 (US4):**
- T043, T45 can run in parallel

**Phase 7 (US5):**
- T050, T52 can run in parallel

**Phase 8 (US6):**
- T058, T59 can run in parallel

**Phase 9 (Polish):**
- T064, T065, T67 can run in parallel
- T068-T071 (browser tests) can all run in parallel

---

## Parallel Example: User Story 1 Tasks

```bash
# Launch parallel CSS tasks:
Task: "Add hero animations to AIdd-book/src/css/animations.css"
Task: "Update global typography in AIdd-book/src/css/custom.css"
Task: "Add visual hierarchy styles to AIdd-book/src/css/custom.css"
Task: "Add feature card images to AIdd-book/static/img/"

# Then sequential component updates:
Task: "Create HeroSection component"
Task: "Update pages/index.tsx to use HeroSection"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006) - Design tokens ready
2. Complete Phase 2: Foundational (T007-T011) - Layout components ready
3. Complete Phase 3: User Story 1 (T012-T024) - Visual redesign complete
4. **STOP and VALIDATE**: Homepage looks professional
5. Deploy/demo if ready - minimal viable redesign complete

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Test visually → Deploy (MVP!)
3. Add User Story 2 + 3 → Test responsiveness + animations → Deploy
4. Add User Story 4-6 → Test performance/a11y/theming → Deploy
5. Polish → Final testing → Ship

### Parallel Team Strategy

With multiple developers after Phase 2:
- Developer A: User Story 1 (visual foundation)
- (After US1) Developer A: User Story 4 (performance)
- (After US1) Developer B: User Story 2 (responsive)
- (After US1) Developer C: User Story 3 (animations)
- (After US1-3) Developer D: User Story 5 (accessibility)
- (After US1-3) Developer E: User Story 6 (theming)

---

## Summary

| Phase | Story | Task Count | Parallel Tasks |
|-------|-------|------------|----------------|
| 1 | Setup | 6 | 2 |
| 2 | Foundational | 5 | 2 |
| 3 | US1 - Visual Appeal | 13 | 4 |
| 4 | US2 - Responsive | 9 | 3 |
| 5 | US3 - Animations | 8 | 3 |
| 6 | US4 - Performance | 7 | 2 |
| 7 | US5 - Accessibility | 8 | 2 |
| 8 | US6 - Dark Mode | 7 | 2 |
| 9 | Polish | 12 | 6 |
| **Total** | | **75** | **26** |

### Independent Test Criteria

| Story | Test Criteria |
|-------|---------------|
| US1 | Homepage visual consistency, professional appearance |
| US2 | All pages work at 320px, 768px, 1024px, 1440px |
| US3 | All transitions complete within 300ms |
| US4 | Lighthouse Performance 90+, LCP < 2.5s |
| US5 | Lighthouse Accessibility 90+, keyboard navigation works |
| US6 | Both themes render correctly, colors switch properly |

### Suggested MVP Scope

**Minimum**: Phase 1-3 (Setup + Foundational + User Story 1) = 24 tasks
- Establishes design tokens
- Creates core layout components
- Delivers professional visual redesign of homepage

**Recommended**: Phase 1-5 (Add US2 + US3) = 41 tasks
- Adds responsive design
- Adds smooth animations
- Complete user-facing experience

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Run `npm run build` to verify no build errors after each phase
- Stop at any checkpoint to validate story independently
