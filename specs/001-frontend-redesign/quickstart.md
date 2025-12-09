# Quickstart: Modern Frontend Redesign

**Feature Branch**: `001-frontend-redesign`
**Date**: 2025-12-09

## Prerequisites

- Node.js >= 20.0
- npm or yarn
- Git

## Getting Started

### 1. Clone and Checkout

```bash
git checkout 001-frontend-redesign
cd AIdd-book
npm install
```

### 2. Start Development Server

```bash
npm run start
```

Open http://localhost:3000 to view the site.

### 3. Project Structure

```
AIdd-book/
├── src/
│   ├── css/
│   │   ├── custom.css      # Global styles + token imports
│   │   ├── tokens.css      # Design token definitions (NEW)
│   │   └── animations.css  # Animation keyframes (NEW)
│   ├── components/
│   │   ├── HeroSection/    # Homepage hero (NEW)
│   │   ├── HomepageFeatures/
│   │   ├── CallToAction/
│   │   └── common/         # Shared utilities (NEW)
│   └── pages/
│       ├── index.tsx       # Homepage
│       └── index.module.css
├── docusaurus.config.ts    # Docusaurus configuration
└── package.json
```

## Design Token Usage

### Import Tokens

Tokens are automatically available via custom.css. Use them in your CSS:

```css
.myComponent {
  color: var(--token-color-text);
  padding: var(--token-spacing-md);
  transition: all var(--token-duration-normal) var(--token-easing-default);
}
```

### Infima Integration

Design tokens map to Infima variables in custom.css:

```css
:root {
  --ifm-color-primary: var(--token-color-primary);
  --ifm-spacing-horizontal: var(--token-spacing-md);
  /* etc. */
}
```

## Component Development

### Creating a New Component

1. Create component directory:
   ```bash
   mkdir -p src/components/MyComponent
   ```

2. Create files:
   - `index.tsx` - Component logic
   - `styles.module.css` - Component styles

3. Follow the component API contract in `specs/001-frontend-redesign/contracts/component-api.ts`

### Example Component

```tsx
// src/components/MyComponent/index.tsx
import type { ReactNode } from 'react';
import styles from './styles.module.css';

interface MyComponentProps {
  children: ReactNode;
  variant?: 'default' | 'highlighted';
}

export default function MyComponent({
  children,
  variant = 'default'
}: MyComponentProps): ReactNode {
  return (
    <div className={`${styles.container} ${styles[variant]}`}>
      {children}
    </div>
  );
}
```

```css
/* src/components/MyComponent/styles.module.css */
.container {
  padding: var(--token-spacing-md);
  border-radius: var(--token-radius-md);
  transition: all var(--token-duration-normal) var(--token-easing-default);
}

.default {
  background: var(--token-color-surface);
}

.highlighted {
  background: var(--token-color-primary-lightest);
}
```

## Animation Guidelines

### Respecting User Preferences

Always wrap animations in motion preference query:

```css
@media (prefers-reduced-motion: no-preference) {
  .animated {
    animation: fadeIn var(--token-duration-slow) var(--token-easing-out);
  }
}
```

### Standard Durations

| Use Case | Token | Value |
|----------|-------|-------|
| Hover states | `--token-duration-fast` | 150ms |
| State changes | `--token-duration-normal` | 200ms |
| Content reveals | `--token-duration-slow` | 300ms |
| Page transitions | `--token-duration-slower` | 500ms |

## Testing

### Visual Testing

```bash
npm run build
npm run serve
```

Then test at:
- Mobile: 375px, 414px
- Tablet: 768px
- Desktop: 1280px, 1440px

### Accessibility Testing

1. Run Lighthouse accessibility audit
2. Test keyboard navigation (Tab, Enter, Escape)
3. Test with screen reader (VoiceOver, NVDA)
4. Verify color contrast with browser DevTools

### Performance Testing

```bash
npm run build
npx lighthouse http://localhost:3000 --view
```

Target scores:
- Performance: 90+
- Accessibility: 90+
- Best Practices: 90+
- SEO: 90+

## Common Tasks

### Modify Color Palette

Edit `src/css/tokens.css` (once created) or the token definitions in `custom.css`:

```css
:root {
  --token-color-primary: #your-new-color;
  /* Update all related shades */
}
```

### Add New Breakpoint

Add to tokens and create corresponding media query:

```css
:root {
  --token-breakpoint-custom: 1600px;
}

@media (min-width: 1600px) {
  /* Custom styles */
}
```

### Swizzle Theme Component

```bash
npm run swizzle @docusaurus/theme-classic ComponentName -- --wrap
```

Wrapping is preferred over ejecting for easier upgrades.

## Troubleshooting

### Styles Not Applying

1. Check CSS Module import syntax
2. Verify custom property names
3. Clear Docusaurus cache: `npm run clear`

### Dark Mode Issues

1. Ensure variables are defined in `[data-theme='dark']`
2. Check for hardcoded colors
3. Test with browser dark mode toggle

### Build Errors

1. Run `npm run typecheck` for TypeScript errors
2. Check for missing imports
3. Verify all paths are correct

## Resources

- [Docusaurus Styling Guide](https://docusaurus.io/docs/styling-layout)
- [Infima Documentation](https://infima.dev/)
- [CSS Custom Properties](https://developer.mozilla.org/en-US/docs/Web/CSS/Using_CSS_custom_properties)
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
