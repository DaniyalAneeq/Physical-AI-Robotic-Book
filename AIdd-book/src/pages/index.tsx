import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title" style={{fontSize: '4rem', color: 'white', zIndex: 1}}>
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle" style={{fontSize: '1.5rem', color: 'white', zIndex: 1, margin: '2rem 0'}}>An Open-Source Textbook on AI-Driven Development in Robotics</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            View Chapters
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="https://github.com/your-org/your-repo">
            View on Github
          </Link>
        </div>
      </div>
    </header>
  );
}

import CallToAction from '@site/src/components/CallToAction';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`AI-Driven Development in Robotics`}
      description="An Open-Source Textbook on AI-Driven Development in Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <CallToAction />
      </main>
    </Layout>
  );
}
