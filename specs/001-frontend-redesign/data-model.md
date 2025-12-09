# Data Model: Modern Frontend Redesign

**Feature Branch**: `001-frontend-redesign`
**Date**: 2025-12-09

## Overview

This feature is primarily a presentation-layer redesign with no backend data storage requirements. The "data model" consists of design tokens, component props, and configuration structures.

## Design Token Schema

### Color Tokens

```typescript
interface ColorTokens {
  // Primary palette
  primary: string;         // Main brand color
  primaryDark: string;     // Darker shade for hover/active
  primaryDarker: string;   // Even darker shade
  primaryDarkest: string;  // Darkest shade
  primaryLight: string;    // Lighter shade
  primaryLighter: string;  // Even lighter shade
  primaryLightest: string; // Lightest shade

  // Semantic colors
  success: string;
  warning: string;
  danger: string;
  info: string;

  // Neutral colors
  background: string;
  backgroundSecondary: string;
  surface: string;
  surfaceElevated: string;
  text: string;
  textSecondary: string;
  textMuted: string;
  border: string;
}

interface ThemeColors {
  light: ColorTokens;
  dark: ColorTokens;
}
```

### Spacing Tokens

```typescript
interface SpacingTokens {
  base: string;      // 4px - base unit
  xs: string;        // 4px
  sm: string;        // 8px
  md: string;        // 16px
  lg: string;        // 24px
  xl: string;        // 32px
  '2xl': string;     // 48px
  '3xl': string;     // 64px
  '4xl': string;     // 96px
}
```

### Typography Tokens

```typescript
interface TypographyTokens {
  fontFamily: {
    base: string;    // System font stack
    heading: string; // Heading font
    mono: string;    // Code font
  };
  fontSize: {
    xs: string;      // 12px
    sm: string;      // 14px
    base: string;    // 16px
    lg: string;      // 18px
    xl: string;      // 20px
    '2xl': string;   // 24px
    '3xl': string;   // 30px
    '4xl': string;   // 36px
    '5xl': string;   // 48px
  };
  fontWeight: {
    normal: number;  // 400
    medium: number;  // 500
    semibold: number;// 600
    bold: number;    // 700
  };
  lineHeight: {
    tight: number;   // 1.25
    normal: number;  // 1.5
    relaxed: number; // 1.75
  };
}
```

### Animation Tokens

```typescript
interface AnimationTokens {
  duration: {
    fast: string;     // 150ms
    normal: string;   // 200ms
    slow: string;     // 300ms
    slower: string;   // 500ms
  };
  easing: {
    default: string;  // ease-in-out
    in: string;       // ease-in
    out: string;      // ease-out
    bounce: string;   // cubic-bezier(0.68, -0.55, 0.265, 1.55)
  };
}
```

### Shadow Tokens

```typescript
interface ShadowTokens {
  sm: string;    // Subtle elevation
  md: string;    // Card elevation
  lg: string;    // Modal/dropdown elevation
  xl: string;    // High emphasis
}
```

## Component Prop Interfaces

### HeroSection Props

```typescript
interface HeroSectionProps {
  title: string;
  subtitle: string;
  backgroundImage?: string;
  backgroundGradient?: string;
  primaryAction?: {
    label: string;
    href: string;
  };
  secondaryAction?: {
    label: string;
    href: string;
  };
  alignment?: 'left' | 'center' | 'right';
}
```

### FeatureCard Props

```typescript
interface FeatureCardProps {
  title: string;
  description: string;
  icon?: React.ReactNode;
  image?: string;
  link?: {
    label: string;
    href: string;
  };
  variant?: 'default' | 'elevated' | 'outlined';
}
```

### SectionContainer Props

```typescript
interface SectionContainerProps {
  children: React.ReactNode;
  background?: 'default' | 'muted' | 'primary' | 'gradient';
  padding?: 'sm' | 'md' | 'lg' | 'xl';
  maxWidth?: 'sm' | 'md' | 'lg' | 'xl' | 'full';
  className?: string;
}
```

### CallToAction Props

```typescript
interface CallToActionProps {
  title: string;
  subtitle?: string;
  primaryButton: {
    label: string;
    href: string;
  };
  secondaryButton?: {
    label: string;
    href: string;
  };
  background?: 'default' | 'gradient' | 'image';
  backgroundImage?: string;
}
```

## Configuration Structures

### Docusaurus Theme Config Extensions

```typescript
interface ExtendedThemeConfig {
  colorMode: {
    defaultMode: 'light' | 'dark';
    disableSwitch: boolean;
    respectPrefersColorScheme: boolean;
  };
  // Custom extension for design system
  designSystem?: {
    enableAnimations: boolean;
    borderRadius: 'none' | 'sm' | 'md' | 'lg';
    spacing: 'compact' | 'normal' | 'relaxed';
  };
}
```

## State Management

No complex state management needed. Component state is local:

| Component | Local State |
|-----------|-------------|
| Navbar | `isExpanded` (mobile menu), `scrolled` (scroll state) |
| ThemeToggle | None (uses Docusaurus context) |
| FeatureCard | `isHovered` (for animations) |

## Validation Rules

### Color Contrast Requirements

| Element Type | Minimum Contrast | Standard |
|--------------|-----------------|----------|
| Body text | 4.5:1 | WCAG AA |
| Large text (18px+) | 3:1 | WCAG AA |
| UI components | 3:1 | WCAG AA |
| Focus indicators | 3:1 | WCAG AA |

### Responsive Breakpoints

| Name | Min Width | Max Width |
|------|-----------|-----------|
| mobile | 0 | 767px |
| tablet | 768px | 1023px |
| laptop | 1024px | 1439px |
| desktop | 1440px | - |

## Entity Relationships

```
Theme
├── ColorTokens
│   ├── light
│   └── dark
├── SpacingTokens
├── TypographyTokens
├── AnimationTokens
└── ShadowTokens

Page
├── HeroSection (1)
├── SectionContainer (many)
│   └── FeatureCard (many)
└── CallToAction (1)
```
