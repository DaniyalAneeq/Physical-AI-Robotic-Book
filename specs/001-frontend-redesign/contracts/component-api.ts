/**
 * Component API Contract
 * Feature: 001-frontend-redesign
 *
 * This file defines TypeScript interfaces for all custom components.
 * Components MUST implement these interfaces exactly.
 */

import type { ReactNode, CSSProperties } from 'react';

// ========================================
// SHARED TYPES
// ========================================

export type ButtonVariant = 'primary' | 'secondary' | 'outline' | 'ghost';
export type Size = 'sm' | 'md' | 'lg' | 'xl';
export type Alignment = 'left' | 'center' | 'right';
export type BackgroundVariant = 'default' | 'muted' | 'primary' | 'gradient' | 'image';

export interface LinkConfig {
  label: string;
  href: string;
  external?: boolean;
}

// ========================================
// HERO SECTION
// ========================================

export interface HeroSectionProps {
  /** Main heading text */
  title: string;
  /** Subtitle or tagline */
  subtitle: string;
  /** Optional background image URL */
  backgroundImage?: string;
  /** Optional CSS gradient string */
  backgroundGradient?: string;
  /** Primary call-to-action button */
  primaryAction?: LinkConfig;
  /** Secondary call-to-action button */
  secondaryAction?: LinkConfig;
  /** Text alignment */
  alignment?: Alignment;
  /** Additional CSS class */
  className?: string;
  /** Inline styles */
  style?: CSSProperties;
}

// ========================================
// FEATURE CARD
// ========================================

export interface FeatureCardProps {
  /** Card title */
  title: string;
  /** Card description */
  description: string;
  /** Icon element (SVG or component) */
  icon?: ReactNode;
  /** Image URL */
  image?: string;
  /** Image alt text (required if image is provided) */
  imageAlt?: string;
  /** Optional link */
  link?: LinkConfig;
  /** Card style variant */
  variant?: 'default' | 'elevated' | 'outlined';
  /** Additional CSS class */
  className?: string;
}

// ========================================
// SECTION CONTAINER
// ========================================

export interface SectionContainerProps {
  /** Content to render inside the section */
  children: ReactNode;
  /** Background style */
  background?: BackgroundVariant;
  /** Background image (only when background='image') */
  backgroundImage?: string;
  /** Vertical padding size */
  padding?: Size;
  /** Maximum content width */
  maxWidth?: Size | 'full';
  /** HTML element to render as */
  as?: 'section' | 'div' | 'article' | 'aside';
  /** ARIA label for the section */
  ariaLabel?: string;
  /** Section ID for anchor links */
  id?: string;
  /** Additional CSS class */
  className?: string;
}

// ========================================
// CALL TO ACTION
// ========================================

export interface CallToActionProps {
  /** Heading text */
  title: string;
  /** Optional subtitle */
  subtitle?: string;
  /** Primary button (required) */
  primaryButton: LinkConfig;
  /** Optional secondary button */
  secondaryButton?: LinkConfig;
  /** Background style */
  background?: 'default' | 'gradient' | 'image';
  /** Background image (only when background='image') */
  backgroundImage?: string;
  /** Text alignment */
  alignment?: Alignment;
  /** Additional CSS class */
  className?: string;
}

// ========================================
// ANIMATED ELEMENT (utility wrapper)
// ========================================

export type AnimationType = 'fadeIn' | 'slideUp' | 'slideIn' | 'scaleIn' | 'none';

export interface AnimatedElementProps {
  /** Content to animate */
  children: ReactNode;
  /** Animation type */
  animation?: AnimationType;
  /** Delay before animation starts (ms) */
  delay?: number;
  /** Animation duration (ms) */
  duration?: number;
  /** Whether to trigger on scroll into view */
  triggerOnScroll?: boolean;
  /** Threshold for scroll trigger (0-1) */
  scrollThreshold?: number;
  /** Additional CSS class */
  className?: string;
}

// ========================================
// FEATURE LIST (Homepage)
// ========================================

export interface FeatureItem {
  title: string;
  description: string;
  icon?: ReactNode;
  image?: string;
  link?: LinkConfig;
}

export interface HomepageFeaturesProps {
  /** Array of features to display */
  features: FeatureItem[];
  /** Number of columns on desktop */
  columns?: 2 | 3 | 4;
  /** Section background */
  background?: BackgroundVariant;
  /** Section title (optional) */
  sectionTitle?: string;
  /** Section subtitle (optional) */
  sectionSubtitle?: string;
  /** Additional CSS class */
  className?: string;
}

// ========================================
// BUTTON (extended from Docusaurus)
// ========================================

export interface ButtonProps {
  /** Button text */
  children: ReactNode;
  /** Click handler (for button behavior) */
  onClick?: () => void;
  /** Link href (renders as link) */
  href?: string;
  /** Button variant */
  variant?: ButtonVariant;
  /** Button size */
  size?: Size;
  /** Full width button */
  fullWidth?: boolean;
  /** Disabled state */
  disabled?: boolean;
  /** Loading state */
  loading?: boolean;
  /** Icon before text */
  iconBefore?: ReactNode;
  /** Icon after text */
  iconAfter?: ReactNode;
  /** Additional CSS class */
  className?: string;
  /** Accessible label (for icon-only buttons) */
  ariaLabel?: string;
}

// ========================================
// VALIDATION HELPERS
// ========================================

/**
 * Runtime validation for required props when image is provided
 */
export function validateFeatureCardProps(props: FeatureCardProps): void {
  if (props.image && !props.imageAlt) {
    console.warn(
      '[FeatureCard] imageAlt is required when image is provided for accessibility'
    );
  }
}

/**
 * Runtime validation for CallToAction
 */
export function validateCallToActionProps(props: CallToActionProps): void {
  if (props.background === 'image' && !props.backgroundImage) {
    console.warn(
      '[CallToAction] backgroundImage is required when background is "image"'
    );
  }
}
