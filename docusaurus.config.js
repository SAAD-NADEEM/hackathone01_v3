// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers
// to scan the config for errors

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'A Comprehensive Educational Resource for Graduate-Level Engineering Students',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io', // TODO: Update with actual GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages or Vercel, this should be /
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // TODO: Update with your GitHub organization/username
  projectName: 'hackathone01_v3', // TODO: Update with your GitHub repository name
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/your-repo/edit/main/',
        },
        blog: false, // Disable blog for this educational book
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg', // TODO: Add actual social card image
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Educational Logo',
          src: 'img/logo.svg', // TODO: Add actual logo
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/your-username/your-repo',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System (ROS 2)',
                to: '/docs/modules/module-1-ros2/intro',
              },
              {
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
                to: '/docs/modules/module-2-digital-twin/intro',
              },
              {
                label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
                to: '/docs/modules/module-3-ai-brain/intro',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/docs/modules/module-4-vla/intro',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
              {
                label: 'Notation',
                to: '/docs/notation',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/your-repo',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Educational Resource. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;