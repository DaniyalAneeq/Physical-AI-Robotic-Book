import {useEffect, type ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
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
        title="Starts Your Robotic Journey"
        subtitle="An Open-Source Textbook on Physical AI & Humanoid Robotics Course"
        backgroundVideo="/video/Video_Editing_and_Generation.mp4"
        primaryAction={{
          label: 'Read Now →',
          href: '/docs/intro',
        }}
        alignment="center"
      />
      <main>
        <HomepageFeatures />
        <CallToAction />
      </main>
    </Layout>
  );
}
