/**
 * LearningPath Component
 * Visual timeline showing the learning journey through the course.
 */

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface PathStep {
  number: number;
  title: string;
  description: string;
  duration: string;
  skills: string[];
}

const learningPath: PathStep[] = [
  {
    number: 1,
    title: 'Foundation',
    description: 'Start with ROS 2 fundamentals - nodes, topics, services, and actions. Build your first robot control systems.',
    duration: '2-3 weeks',
    skills: ['ROS 2 Architecture', 'Python rclpy', 'Publisher/Subscriber', 'Services & Actions'],
  },
  {
    number: 2,
    title: 'Simulation',
    description: 'Create digital twins in Gazebo. Learn URDF modeling, sensor simulation, and physics-based testing.',
    duration: '2-3 weeks',
    skills: ['Gazebo Sim', 'URDF/SDF', 'Sensor Plugins', 'Robot Modeling'],
  },
  {
    number: 3,
    title: 'AI Integration',
    description: 'Master NVIDIA Isaac Sim for AI training. Implement reinforcement learning and computer vision.',
    duration: '3-4 weeks',
    skills: ['Isaac Sim', 'RL Training', 'Domain Randomization', 'Synthetic Data'],
  },
  {
    number: 4,
    title: 'Advanced AI',
    description: 'Implement Vision-Language-Action models. Build robots that understand and act on natural language.',
    duration: '3-4 weeks',
    skills: ['VLA Models', 'Multimodal AI', 'LLM Integration', 'End-to-End Learning'],
  },
];

function PathCard({ step, isLast }: { step: PathStep; isLast: boolean }) {
  return (
    <div className={styles.pathStep}>
      <div className={styles.stepTimeline}>
        <div className={styles.stepNumber}>{step.number}</div>
        {!isLast && <div className={styles.stepConnector} />}
      </div>
      <div className={styles.stepContent}>
        <div className={styles.stepHeader}>
          <Heading as="h3" className={styles.stepTitle}>
            {step.title}
          </Heading>
          <span className={styles.stepDuration}>{step.duration}</span>
        </div>
        <p className={styles.stepDescription}>{step.description}</p>
        <div className={styles.skillTags}>
          {step.skills.map((skill) => (
            <span key={skill} className={styles.skillTag}>
              {skill}
            </span>
          ))}
        </div>
      </div>
    </div>
  );
}

export default function LearningPath(): React.ReactElement {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <div className={styles.header}>
          <Heading as="h2" className={styles.sectionTitle}>
            Your Learning Journey
          </Heading>
          <p className={styles.sectionSubtitle}>
            A structured path from robotics fundamentals to cutting-edge AI applications
          </p>
        </div>
        <div className={styles.pathContainer}>
          {learningPath.map((step, index) => (
            <PathCard
              key={step.number}
              step={step}
              isLast={index === learningPath.length - 1}
            />
          ))}
        </div>
        <div className={styles.ctaContainer}>
          <Link
            className={clsx('button', 'button--lg', styles.ctaButton)}
            to="/docs/intro"
          >
            Start Your Journey
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M5 12h14" />
              <path d="m12 5 7 7-7 7" />
            </svg>
          </Link>
        </div>
      </div>
    </section>
  );
}
