// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'modules/module-1-ros2/intro',
      },
      items: [
        'modules/module-1-ros2/chapter-1',
        'modules/module-1-ros2/chapter-2',
        'modules/module-1-ros2/chapter-3',
        'modules/module-1-ros2/chapter-4',
        'modules/module-1-ros2/chapter-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'doc',
        id: 'modules/module-2-digital-twin/intro',
      },
      items: [
        'modules/module-2-digital-twin/chapter-6',
        'modules/module-2-digital-twin/chapter-7',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {
        type: 'doc',
        id: 'modules/module-3-ai-brain/intro',
      },
      items: [
        'modules/module-3-ai-brain/chapter-8',
        'modules/module-3-ai-brain/chapter-9',
        'modules/module-3-ai-brain/chapter-10',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'doc',
        id: 'modules/module-4-vla/intro',
      },
      items: [
        'modules/module-4-vla/chapter-11',
        'modules/module-4-vla/chapter-12',
        'modules/module-4-vla/chapter-13',
      ],
    },
    {
      type: 'category',
      label: 'Additional Resources',
      items: [
        'glossary',
        'notation',
        'capstone-project',
      ],
    },
  ],
};

module.exports = sidebars;