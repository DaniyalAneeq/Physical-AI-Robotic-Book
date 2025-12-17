import {useEffect, type ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ModuleOverview from '@site/src/components/ModuleOverview';
import TechStack from '@site/src/components/TechStack';
import Stats from '@site/src/components/Stats';
import LearningPath from '@site/src/components/LearningPath';
import CallToAction from '@site/src/components/CallToAction';

function useTransparentNavbar() {
  useEffect(() => {
    const navbar = document.querySelector('.navbar');
    if (!navbar) return;

    // Add transparent class initially
    navbar.classList.add('navbar--hero-overlay');

    const handleScroll = () => {
      if (window.scrollY > 50) {
        navbar.classList.add('navbar--scrolled');
      } else {
        navbar.classList.remove('navbar--scrolled');
      }
    };

    window.addEventListener('scroll', handleScroll, {passive: true});
    handleScroll(); // Check initial scroll position

    return () => {
      navbar.classList.remove('navbar--hero-overlay', 'navbar--scrolled');
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  useTransparentNavbar();

  return (
    <Layout
      title="AI-Driven Development in Robotics"
      description="An Open-Source Textbook on AI-Driven Development in Robotics">
      <HeroSection
        title="Start Your Robotics Journey"
        subtitle="An Open-Source Textbook on Physical AI & Humanoid Robotics"
        backgroundVideo="/video/Video_Editing_and_Generation.mp4"
        primaryAction={{
          label: 'Start Learning',
          href: '/docs/intro',
        }}
        secondaryAction={{
          label: 'View on GitHub',
          href: 'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book',
          external: true,
        }}
        alignment="center"
      />
      <main>
        <HomepageFeatures />
        <ModuleOverview />
        <Stats />
        <TechStack />
        <LearningPath />
        <CallToAction
          title="Ready to Build Intelligent Robots?"
          subtitle="Join thousands of learners mastering AI-driven robotics development. Start your journey today with our comprehensive, hands-on curriculum."
          primaryButton={{
            label: 'Start Reading Free',
            href: '/docs/intro',
          }}
          secondaryButton={{
            label: 'View GitHub',
            href: 'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book',
            external: true,
          }}
        />
      </main>
    </Layout>
  );
}
