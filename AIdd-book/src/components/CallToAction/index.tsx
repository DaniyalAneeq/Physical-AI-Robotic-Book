import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

export default function CallToAction(): ReactNode {
  return (
    <section className={styles.callToAction}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <Heading as="h2" className={styles.title}>
              Ready to Dive In?
            </Heading>
            <p className={styles.subtitle}>
              Start learning about AI-driven development in robotics today.
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Read the First Chapter
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
