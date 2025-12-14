/**
 * TechStack Component
 * Displays the technologies covered in the course with animated logos.
 */

import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface Technology {
  name: string;
  icon: React.ReactNode;
  category: string;
}

// Technology Icons
function ROSIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M24 4C12.95 4 4 12.95 4 24s8.95 20 20 20 20-8.95 20-20S35.05 4 24 4zm0 36c-8.82 0-16-7.18-16-16S15.18 8 24 8s16 7.18 16 16-7.18 16-16 16zm-4-20a4 4 0 1 0 0-8 4 4 0 0 0 0 8zm8 0a4 4 0 1 0 0-8 4 4 0 0 0 0 8zm-4 12a4 4 0 1 0 0-8 4 4 0 0 0 0 8z"/>
    </svg>
  );
}

function PythonIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M24 4c-5.5 0-10.3 0.4-14 1.2V18c0 2.2 1.8 4 4 4h20c2.2 0 4 1.8 4 4v12.8c3.7-0.8 6-1.8 6-3.8V10c0-4-9-6-20-6zm-4 8a2 2 0 1 1 0-4 2 2 0 0 1 0 4z"/>
      <path d="M24 44c5.5 0 10.3-0.4 14-1.2V30c0-2.2-1.8-4-4-4H14c-2.2 0-4-1.8-4-4V9.2C6.3 10 4 11 4 13v25c0 4 9 6 20 6zm4-8a2 2 0 1 1 0 4 2 2 0 0 1 0-4z"/>
    </svg>
  );
}

function NvidiaIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M19.6 17.2v13.6c0 0 2.2-0.2 3.4-0.4 5.4-0.8 10-4 10-9.6 0-5.6-4.8-9-10.2-9-1.2 0-3.2 0.2-3.2 0.2v-5.2c0 0 1.8-0.2 3.4-0.2 8.4 0 15.2 5.4 15.2 14.2s-6.6 14.6-15 15.4c-1.8 0.2-3.6 0.2-3.6 0.2V41h-5.2V17.2h5.2z"/>
    </svg>
  );
}

function GazeboIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M24 4l-16 8v24l16 8 16-8V12L24 4zm0 4l12 6-12 6-12-6 12-6zM10 16.5l12 6v15l-12-6v-15zm28 0v15l-12 6v-15l12-6z"/>
    </svg>
  );
}

function UnityIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M38 24l-8-14h-12l-8 14 8 14h12l8-14zm-10-10l6 10-6 10h-8l-6-10 6-10h8z"/>
      <path d="M28 18v12h-8V18h8zm-4 2h-2v8h2v-8z"/>
    </svg>
  );
}

function AIIcon() {
  return (
    <svg viewBox="0 0 48 48" fill="currentColor">
      <path d="M24 4C12.95 4 4 12.95 4 24s8.95 20 20 20 20-8.95 20-20S35.05 4 24 4zm0 6c7.73 0 14 6.27 14 14s-6.27 14-14 14-14-6.27-14-14 6.27-14 14-14zm0 4c-5.52 0-10 4.48-10 10s4.48 10 10 10 10-4.48 10-10-4.48-10-10-10zm0 4c3.31 0 6 2.69 6 6s-2.69 6-6 6-6-2.69-6-6 2.69-6 6-6z"/>
    </svg>
  );
}

const technologies: Technology[] = [
  { name: 'ROS 2', icon: <ROSIcon />, category: 'Robotics' },
  { name: 'Python', icon: <PythonIcon />, category: 'Language' },
  { name: 'NVIDIA Isaac', icon: <NvidiaIcon />, category: 'Simulation' },
  { name: 'Gazebo', icon: <GazeboIcon />, category: 'Digital Twin' },
  { name: 'Unity', icon: <UnityIcon />, category: 'Simulation' },
  { name: 'Machine Learning', icon: <AIIcon />, category: 'AI/ML' },
];

function TechCard({ tech, index }: { tech: Technology; index: number }) {
  return (
    <div
      className={styles.techCard}
      style={{ animationDelay: `${index * 0.1}s` }}
    >
      <div className={styles.techIcon}>
        {tech.icon}
      </div>
      <span className={styles.techName}>{tech.name}</span>
      <span className={styles.techCategory}>{tech.category}</span>
    </div>
  );
}

export default function TechStack(): React.ReactElement {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <div className={styles.header}>
          <Heading as="h2" className={styles.sectionTitle}>
            Technologies You'll Master
          </Heading>
          <p className={styles.sectionSubtitle}>
            Industry-standard tools and frameworks used by leading robotics companies
          </p>
        </div>
        <div className={styles.techGrid}>
          {technologies.map((tech, index) => (
            <TechCard key={tech.name} tech={tech} index={index} />
          ))}
        </div>
      </div>
    </section>
  );
}
