/**
 * ModuleOverview Component
 * Displays the 4 modules of the course in an attractive grid layout.
 */

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface Module {
  number: number;
  title: string;
  description: string;
  icon: React.ReactNode;
  href: string;
  color: string;
}

// Module Icons
function ROS2Icon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="12" r="3" />
      <path d="M12 2v4" />
      <path d="M12 18v4" />
      <path d="m4.93 4.93 2.83 2.83" />
      <path d="m16.24 16.24 2.83 2.83" />
      <path d="M2 12h4" />
      <path d="M18 12h4" />
      <path d="m4.93 19.07 2.83-2.83" />
      <path d="m16.24 7.76 2.83-2.83" />
    </svg>
  );
}

function DigitalTwinIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <rect x="2" y="3" width="20" height="14" rx="2" />
      <path d="M8 21h8" />
      <path d="M12 17v4" />
      <path d="M7 8h.01" />
      <path d="M17 8h.01" />
      <path d="M7 12h10" />
    </svg>
  );
}

function IsaacIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <path d="M12 2a4 4 0 0 1 4 4v2a4 4 0 0 1-8 0V6a4 4 0 0 1 4-4z" />
      <path d="M10 9v2" />
      <path d="M14 9v2" />
      <path d="M9 16h6" />
      <rect x="5" y="12" width="14" height="10" rx="2" />
    </svg>
  );
}

function VLAIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="12" r="10" />
      <path d="M12 2a14.5 14.5 0 0 0 0 20 14.5 14.5 0 0 0 0-20" />
      <path d="M2 12h20" />
      <path d="M12 2c-2.5 3-4 6.5-4 10s1.5 7 4 10c2.5-3 4-6.5 4-10s-1.5-7-4-10" />
    </svg>
  );
}

const modules: Module[] = [
  {
    number: 1,
    title: 'ROS 2 Fundamentals',
    description: 'Master Robot Operating System 2 - the foundation of modern robotics development with nodes, topics, and services.',
    icon: <ROS2Icon />,
    href: '/docs/module1/chapter1',
    color: 'blue',
  },
  {
    number: 2,
    title: 'Digital Twins',
    description: 'Create virtual replicas of physical robots using Gazebo for simulation, testing, and development.',
    icon: <DigitalTwinIcon />,
    href: '/docs/module2/chapter1-intro-digital-twins',
    color: 'green',
  },
  {
    number: 3,
    title: 'NVIDIA Isaac Sim',
    description: 'Build the AI brain of your robot with NVIDIA\'s powerful simulation platform for reinforcement learning.',
    icon: <IsaacIcon />,
    href: '/docs/category/module-3-the-ai-robot-brain-nvidia-isaac',
    color: 'purple',
  },
  {
    number: 4,
    title: 'Vision-Language-Action',
    description: 'Implement cutting-edge VLA models that combine vision, language understanding, and robot control.',
    icon: <VLAIcon />,
    href: '/docs/category/module-4-vision-language-action-vla',
    color: 'orange',
  },
];

function ModuleCard({ module }: { module: Module }) {
  return (
    <Link to={module.href} className={clsx(styles.moduleCard, styles[`module${module.color}`])}>
      <div className={styles.moduleNumber}>Module {module.number}</div>
      <div className={styles.moduleIcon}>{module.icon}</div>
      <Heading as="h3" className={styles.moduleTitle}>
        {module.title}
      </Heading>
      <p className={styles.moduleDescription}>{module.description}</p>
      <span className={styles.moduleLink}>
        Start Learning
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M5 12h14" />
          <path d="m12 5 7 7-7 7" />
        </svg>
      </span>
    </Link>
  );
}

export default function ModuleOverview(): React.ReactElement {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <div className={styles.header}>
          <Heading as="h2" className={styles.sectionTitle}>
            Course Modules
          </Heading>
          <p className={styles.sectionSubtitle}>
            A comprehensive journey from robotics fundamentals to cutting-edge AI integration
          </p>
        </div>
        <div className={styles.grid}>
          {modules.map((module) => (
            <ModuleCard key={module.number} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}
