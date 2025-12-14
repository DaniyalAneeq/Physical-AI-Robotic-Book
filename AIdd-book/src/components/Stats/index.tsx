/**
 * Stats Component
 * Displays impressive statistics about the course/book with animated counters.
 */

import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface Stat {
  value: string;
  label: string;
  description: string;
  icon: React.ReactNode;
}

// Stat Icons
function ChaptersIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <path d="M4 19.5v-15A2.5 2.5 0 0 1 6.5 2H20v20H6.5a2.5 2.5 0 0 1 0-5H20" />
      <path d="M8 7h8" />
      <path d="M8 11h6" />
    </svg>
  );
}

function CodeIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <polyline points="16 18 22 12 16 6" />
      <polyline points="8 6 2 12 8 18" />
    </svg>
  );
}

function ProjectsIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <rect x="2" y="3" width="20" height="14" rx="2" />
      <path d="M8 21h8" />
      <path d="M12 17v4" />
      <path d="m9 9 3 3 3-3" />
    </svg>
  );
}

function TimeIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="12" r="10" />
      <polyline points="12 6 12 12 16 14" />
    </svg>
  );
}

const stats: Stat[] = [
  {
    value: '4',
    label: 'Comprehensive Modules',
    description: 'From ROS 2 basics to advanced VLA models',
    icon: <ChaptersIcon />,
  },
  {
    value: '20+',
    label: 'Chapters',
    description: 'In-depth coverage of each topic',
    icon: <ChaptersIcon />,
  },
  {
    value: '50+',
    label: 'Code Examples',
    description: 'Real-world, production-ready code',
    icon: <CodeIcon />,
  },
  {
    value: '10+',
    label: 'Hands-on Projects',
    description: 'Build complete robotics applications',
    icon: <ProjectsIcon />,
  },
];

function StatCard({ stat, index }: { stat: Stat; index: number }) {
  return (
    <div
      className={styles.statCard}
      style={{ animationDelay: `${index * 0.15}s` }}
    >
      <div className={styles.statIcon}>
        {stat.icon}
      </div>
      <div className={styles.statContent}>
        <span className={styles.statValue}>{stat.value}</span>
        <span className={styles.statLabel}>{stat.label}</span>
        <span className={styles.statDescription}>{stat.description}</span>
      </div>
    </div>
  );
}

export default function Stats(): React.ReactElement {
  return (
    <section className={styles.section}>
      <div className={styles.background} />
      <div className={styles.container}>
        <div className={styles.header}>
          <Heading as="h2" className={styles.sectionTitle}>
            What's Inside
          </Heading>
          <p className={styles.sectionSubtitle}>
            Everything you need to become a physical AI and robotics expert
          </p>
        </div>
        <div className={styles.statsGrid}>
          {stats.map((stat, index) => (
            <StatCard key={stat.label} stat={stat} index={index} />
          ))}
        </div>
      </div>
    </section>
  );
}
