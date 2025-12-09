import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface LinkConfig {
  label: string;
  href: string;
  external?: boolean;
}

export interface CallToActionProps {
  /** Heading text */
  title?: string;
  /** Optional subtitle */
  subtitle?: string;
  /** Primary button */
  primaryButton?: LinkConfig;
  /** Optional secondary button */
  secondaryButton?: LinkConfig;
  /** Additional CSS class */
  className?: string;
}

export default function CallToAction({
  title = 'Ready to Dive In?',
  subtitle = 'Start learning about AI-driven development in robotics today.',
  primaryButton = { label: 'Read the First Chapter', href: '/docs/intro' },
  secondaryButton,
  className,
}: CallToActionProps): ReactNode {
  return (
    <section className={clsx(styles.callToAction, className)}>
      <div className={styles.container}>
        <Heading as="h2" className={styles.title}>
          {title}
        </Heading>
        <p className={styles.subtitle}>{subtitle}</p>
        <div className={styles.actions}>
          <Link
            className={clsx('button', 'button--lg', styles.primaryButton)}
            to={primaryButton.href}
            {...(primaryButton.external ? { target: '_blank', rel: 'noopener noreferrer' } : {})}
          >
            {primaryButton.label}
          </Link>
          {secondaryButton && (
            <Link
              className={clsx('button', 'button--lg', styles.secondaryButton)}
              to={secondaryButton.href}
              {...(secondaryButton.external ? { target: '_blank', rel: 'noopener noreferrer' } : {})}
            >
              {secondaryButton.label}
            </Link>
          )}
        </div>
      </div>
    </section>
  );
}
