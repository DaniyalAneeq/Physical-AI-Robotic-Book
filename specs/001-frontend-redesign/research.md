# Research: Modern Frontend Redesign

**Feature Branch**: `001-frontend-redesign`
**Date**: 2025-12-09

## Technical Context Analysis

### Current Stack Assessment

| Aspect | Current State | Notes |
|--------|---------------|-------|
| Framework | Docusaurus 3.9.2 | Latest stable, supports Docusaurus v4 future flag enabled |
| React | React 19.0.0 | Latest stable |
| CSS Framework | Infima (built-in) | Docusaurus's default CSS framework |
| Build System | Webpack (via Docusaurus) | Handled by Docusaurus internally |
| TypeScript | 5.6.2 | Enabled for type safety |

### Research Findings

#### 1. Docusaurus Theming Best Practices

**Decision**: Use Docusaurus's built-in theming system with CSS custom properties

**Rationale**:
- Docusaurus uses Infima CSS framework which provides CSS custom properties for theming
- Swizzling theme components allows deep customization while maintaining upgrade compatibility
- CSS Modules are already in use and well-supported

**Alternatives Considered**:
- CSS-in-JS (styled-components, emotion): Rejected - adds bundle size, Infima already provides theming
- Tailwind CSS: Rejected - requires additional configuration, conflicts with Infima
- SASS/SCSS: Considered - would require additional build configuration, CSS custom properties sufficient

#### 2. Design Token System

**Decision**: Implement design tokens via CSS custom properties in custom.css

**Rationale**:
- Native browser support, no runtime cost
- Docusaurus's Infima already uses `--ifm-*` prefix convention
- Easy to override in light/dark themes
- Consistent with existing codebase

**Token Categories**:
- Colors: `--ifm-color-*` (extend existing)
- Spacing: `--ifm-spacing-*` (new)
- Typography: `--ifm-font-*` (extend existing)
- Shadows: `--ifm-shadow-*` (new)
- Borders: `--ifm-border-radius-*` (extend existing)
- Transitions: `--ifm-transition-*` (new)

#### 3. Component Architecture

**Decision**: Swizzle minimal theme components, create custom components in `src/components/`

**Rationale**:
- Swizzling creates copies that must be manually maintained on upgrades
- Custom components in src/ are isolated and composable
- Only swizzle when absolutely necessary (e.g., Navbar, Footer structure)

**Components to Swizzle** (wrap, not eject):
- `@theme/Navbar` - for custom styling hooks
- `@theme/Footer` - for layout adjustments

**Custom Components to Create**:
- `FeatureCard` - consistent feature display
- `HeroSection` - animated hero with gradients
- `SectionContainer` - consistent section layout
- `AnimatedElement` - reusable animation wrapper

#### 4. Animation Strategy

**Decision**: CSS-based animations with `prefers-reduced-motion` support

**Rationale**:
- No additional dependencies needed
- Better performance than JS animations
- Native reduced motion support
- Already using CSS transitions in existing code

**Animation Guidelines**:
- Hover states: 150-200ms ease-out
- State changes: 200-300ms ease-in-out
- Page transitions: Use Docusaurus's built-in (plugin-based if needed)
- All animations wrapped in `@media (prefers-reduced-motion: no-preference)`

#### 5. Responsive Breakpoints

**Decision**: Use Docusaurus's existing breakpoints, extend as needed

**Rationale**:
- Docusaurus already defines `--ifm-breakpoint-*` values
- Infima grid system is responsive by default
- Consistency with existing layout

**Breakpoints**:
- Mobile: < 768px (existing `996px` is tablet in Docusaurus)
- Tablet: 768px - 1023px
- Laptop: 1024px - 1439px
- Desktop: >= 1440px

#### 6. Performance Optimization

**Decision**: Leverage Docusaurus built-ins, add minimal optimizations

**Rationale**:
- Docusaurus already handles code splitting, lazy loading
- Image optimization via `@docusaurus/plugin-ideal-image` available
- CSS is already extracted and minified

**Optimizations to Add**:
- Replace external image URLs with local optimized images
- Lazy load below-fold images
- Preload critical fonts
- Use `font-display: swap` for web fonts

#### 7. Accessibility Implementation

**Decision**: Enhance existing semantic HTML, add ARIA where needed

**Rationale**:
- Docusaurus templates already use semantic HTML
- Infima provides focus states
- Need to ensure custom components maintain standards

**Focus Areas**:
- Ensure all custom components have proper focus indicators
- Add skip links if not present
- Verify color contrast in custom color palette
- Add ARIA labels to icon-only buttons

## Resolved Unknowns

| Unknown | Resolution |
|---------|------------|
| CSS approach | CSS Modules + custom properties (existing pattern) |
| Animation library | Pure CSS with transitions/keyframes |
| Component library | None - use Docusaurus themes + custom |
| Testing tools | Lighthouse CLI, axe-core for accessibility |
| Build process | Docusaurus built-in (no changes needed) |

## Dependencies to Add

| Package | Purpose | Version |
|---------|---------|---------|
| `@docusaurus/plugin-ideal-image` | Image optimization | ^3.9.2 |

## Files to Modify

| File | Changes |
|------|---------|
| `src/css/custom.css` | Design tokens, global styles |
| `src/pages/index.tsx` | Restructure homepage |
| `src/pages/index.module.css` | Hero section redesign |
| `src/components/HomepageFeatures/` | Component redesign |
| `src/components/CallToAction/` | Component redesign |
| `docusaurus.config.ts` | Theme configuration updates |

## New Files to Create

| File | Purpose |
|------|---------|
| `src/css/tokens.css` | Design token definitions (imported in custom.css) |
| `src/css/animations.css` | Animation keyframes and utilities |
| `src/components/HeroSection/` | New hero component |
| `src/components/SectionContainer/` | Layout wrapper component |
| `src/components/common/` | Shared utility components |
