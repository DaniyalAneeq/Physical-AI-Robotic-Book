import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Building Intelligent Robots',
  favicon: 'img/favicon.svg',

  future: {
    v4: true,
  },

  // Custom fields for chatbot configuration
  customFields: {
    chatbot: {
      apiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000',
      enabled: process.env.CHATBOT_ENABLED !== 'false',
    },
  },

  // GitHub Pages deployment URL
  url: 'https://daniyalaneeq.github.io',

  // Required for GitHub Pages — MUST end with a slash
  baseUrl: '/Physical-AI-Robotic-Book/',

  // GitHub repo details
  organizationName: 'DaniyalAneeq',
  projectName: 'Physical-AI-Robotic-Book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book/tree/main/AIdd-book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book/tree/main/AIdd-book/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    colorMode: {
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/book-logo-optimized.svg',
        srcDark: 'img/book-logo-optimized.svg',
        width: 32,
        height: 32,
      },
      hideOnScroll: false,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          to: '/chat',
          label: '💬 Ask AI',
          position: 'left',
        },
        // { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2 Fundamentals',
              to: '/docs/module1/chapter1',
            },
            {
              label: 'Module 2: Digital Twins',
              to: '/docs/module2/chapter1-intro-digital-twins',
            },
            {
              label: 'Module 3: Isaac Sim',
              to: '/docs/category/module-3-the-ai-robot-brain-nvidia-isaac',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/category/module-4-vision-language-action-vla',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Glossary',
              to: '/docs/glossary',
            },
            {
              label: 'Notation Guide',
              to: '/docs/notation',
            },
            // {
            //   label: 'Blog',
            //   to: '/blog',
            // },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book',
            },
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
          ],
        },
        {
          title: 'About',
          items: [
            {
              label: 'AI Driven Development',
              href: 'https://github.com/DaniyalAneeq',
            },
            {
              label: 'Contribute',
              href: 'https://github.com/DaniyalAneeq/Physical-AI-Robotic-Book/blob/main/CONTRIBUTING.md',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI Driven Development. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
