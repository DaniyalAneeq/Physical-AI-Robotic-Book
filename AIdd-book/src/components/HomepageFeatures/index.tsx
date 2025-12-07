import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Introduction to AI in Robotics',
    Svg: () => <img src="https://picsum.photos/200/200?random=1" alt="AI in Robotics" />,
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
    Svg: () => <img src="https://picsum.photos/200/200?random=2" alt="Hands-On Projects" />,
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
    Svg: () => <img src="https://picsum.photos/200/200?random=3" alt="Cutting-Edge Research" />,
    description: (
      <>
        Explore the latest research and techniques in AI for robotics. This
        book covers a wide range of topics, including machine learning,
        computer vision, and natural language processing.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className="text--center">
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
