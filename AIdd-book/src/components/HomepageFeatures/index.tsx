import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: ReactNode;
  description: ReactNode;
};

// SVG Icons as components
function RobotIcon() {
  return (
    <svg
      className={styles.featureIcon}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect x="3" y="11" width="18" height="10" rx="2" />
      <circle cx="12" cy="5" r="3" />
      <path d="M12 8v3" />
      <circle cx="8" cy="16" r="1" fill="currentColor" />
      <circle cx="16" cy="16" r="1" fill="currentColor" />
    </svg>
  );
}

function CodeIcon() {
  return (
    <svg
      className={styles.featureIcon}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <polyline points="16 18 22 12 16 6" />
      <polyline points="8 6 2 12 8 18" />
      <line x1="12" y1="2" x2="12" y2="22" />
    </svg>
  );
}

function ResearchIcon() {
  return (
    <svg
      className={styles.featureIcon}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M12 2L2 7l10 5 10-5-10-5z" />
      <path d="M2 17l10 5 10-5" />
      <path d="M2 12l10 5 10-5" />
    </svg>
  );
}

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Introduction to AI in Robotics',
    icon: <RobotIcon />,
    description: (
      <>
        Learn the fundamentals of AI and how it can be applied to robotics.
        This book provides a comprehensive introduction to the field,
        covering everything from basic concepts to advanced topics.
      </>
    ),
  },
  {
    title: 'Hands-On Projects and Examples',
    icon: <CodeIcon />,
    description: (
      <>
        Get hands-on experience with AI in robotics through a variety of
        projects and examples. This book includes detailed instructions and
        code samples to help you build your own AI-powered robots.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Research and Techniques',
    icon: <ResearchIcon />,
    description: (
      <>
        Explore the latest research and techniques in AI for robotics. This
        book covers a wide range of topics, including machine learning,
        computer vision, and natural language processing.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureColumn)}>
      <div className={styles.featureCard}>
        {icon}
        <Heading as="h3" className={styles.featureTitle}>
          {title}
        </Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={clsx('row', styles.featuresRow)}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
