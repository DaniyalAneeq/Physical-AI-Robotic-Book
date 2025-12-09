/**
 * SectionContainer Component
 * A reusable wrapper for page sections with configurable background and padding.
 */

import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type Size = 'sm' | 'md' | 'lg' | 'xl';
type BackgroundVariant = 'default' | 'muted' | 'primary' | 'gradient' | 'image';

export interface SectionContainerProps {
  /** Content to render inside the section */
  children: React.ReactNode;
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

export default function SectionContainer({
  children,
  background = 'default',
  backgroundImage,
  padding = 'md',
  maxWidth = 'lg',
  as: Component = 'section',
  ariaLabel,
  id,
  className,
}: SectionContainerProps): React.ReactElement {
  const backgroundStyle = backgroundImage && background === 'image'
    ? { backgroundImage: `url(${backgroundImage})` }
    : undefined;

  return (
    <Component
      id={id}
      aria-label={ariaLabel}
      className={clsx(
        styles.section,
        styles[`bg-${background}`],
        styles[`padding-${padding}`],
        className
      )}
      style={backgroundStyle}
    >
      <div
        className={clsx(
          styles.container,
          styles[`maxWidth-${maxWidth}`]
        )}
      >
        {children}
      </div>
    </Component>
  );
}
