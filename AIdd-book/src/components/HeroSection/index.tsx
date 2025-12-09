/**
 * HeroSection Component
 * A reusable hero section with video/gradient background and animated content.
 */

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type Alignment = 'left' | 'center' | 'right';

interface LinkConfig {
  label: string;
  href: string;
  external?: boolean;
}

export interface HeroSectionProps {
  /** Main heading text */
  title: string;
  /** Subtitle or tagline */
  subtitle: string;
  /** Optional background video URL */
  backgroundVideo?: string;
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
  style?: React.CSSProperties;
}

export default function HeroSection({
  title,
  subtitle,
  backgroundVideo,
  backgroundImage,
  backgroundGradient,
  primaryAction,
  secondaryAction,
  alignment = 'center',
  className,
  style,
}: HeroSectionProps): React.ReactElement {
  const hasBackground = backgroundImage || backgroundGradient || backgroundVideo;
  const videoSrc = useBaseUrl(backgroundVideo || '');

  const backgroundStyle: React.CSSProperties = {
    ...(backgroundImage && !backgroundVideo ? { backgroundImage: `url(${backgroundImage})` } : {}),
    ...(backgroundGradient && !backgroundImage && !backgroundVideo
      ? { background: backgroundGradient }
      : {}),
    ...style,
  };

  return (
    <header
      className={clsx(
        styles.hero,
        hasBackground && styles.hasBackground,
        backgroundImage && !backgroundVideo && styles.hasImage,
        backgroundVideo && styles.hasVideo,
        styles[`align-${alignment}`],
        className
      )}
      style={backgroundStyle}
    >
      {/* Video Background */}
      {backgroundVideo && (
        <div className={styles.videoWrapper}>
          <video
            className={styles.videoBackground}
            autoPlay
            loop
            muted
            playsInline
            src={videoSrc}
          />
        </div>
      )}

      {/* Dark Overlay */}
      {(backgroundImage || backgroundVideo) && <div className={styles.overlay} />}

      {/* Content */}
      <div className={styles.container}>
        <div className={styles.content}>
          <Heading as="h1" className={styles.title}>
            {title}
          </Heading>
          <p className={styles.subtitle}>{subtitle}</p>
          {(primaryAction || secondaryAction) && (
            <div className={styles.actions}>
              {primaryAction && (
                <Link
                  className={clsx('button', 'button--lg', styles.primaryButton)}
                  to={primaryAction.href}
                  {...(primaryAction.external ? { target: '_blank', rel: 'noopener noreferrer' } : {})}
                >
                  {primaryAction.label}
                </Link>
              )}
              {secondaryAction && (
                <Link
                  className={clsx('button', 'button--lg', styles.secondaryButton)}
                  to={secondaryAction.href}
                  {...(secondaryAction.external ? { target: '_blank', rel: 'noopener noreferrer' } : {})}
                >
                  {secondaryAction.label}
                </Link>
              )}
            </div>
          )}
        </div>
      </div>
    </header>
  );
}
