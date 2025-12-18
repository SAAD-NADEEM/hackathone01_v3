import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            View Curriculum - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModuleCard({ title, description, chapters, to }) {
  return (
    <div className="col col--3">
      <div className="card">
        <div className="card__body">
          <h3>{title}</h3>
          <p>{description}</p>
          <ul>
            {chapters.map((chapter, index) => (
              <li key={index}>{chapter}</li>
            ))}
          </ul>
        </div>
        <div className="card__footer">
          <Link className="button button--primary" to={to}>
            Explore
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  // Define module information
  const modules = [
    {
      title: 'Module 1: The Robotic Nervous System (ROS 2)',
      description: 'Introduction to ROS 2 for robot operating systems',
      chapters: [
        'Chapter 1: ROS 2 Architecture & Components',
        'Chapter 2: Nodes, Topics & Services',
        'Chapter 3: ROS 2 Packages & Launch Files',
        'Chapter 4: TF Transforms & Robot State',
        'Chapter 5: Navigation & Path Planning'
      ],
      to: '/docs/modules/module-1-ros2/intro'
    },
    {
      title: 'Module 2: The Digital Twin (Gazebo & Unity)',
      description: 'Simulation environments for robot development',
      chapters: [
        'Chapter 6: Gazebo Simulation Environment',
        'Chapter 7: Unity Physics & Robotics Simulation'
      ],
      to: '/docs/modules/module-2-digital-twin/intro'
    },
    {
      title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      description: 'AI algorithms for robotic intelligence',
      chapters: [
        'Chapter 8: Perception & Sensing',
        'Chapter 9: Decision Making & Control',
        'Chapter 10: Learning & Adaptation'
      ],
      to: '/docs/modules/module-3-ai-brain/intro'
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Advanced integration of perception, cognition, and action',
      chapters: [
        'Chapter 11: Computer Vision & Robotics',
        'Chapter 12: Natural Language Processing for Robots',
        'Chapter 13: Vision-Language-Action Integration'
      ],
      to: '/docs/modules/module-4-vla/intro'
    }
  ];

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Comprehensive Educational Resource for Graduate-Level Engineering Students">
      <HomepageHeader />
      <main>
        <section className={styles.modules}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2>Course Structure</h2>
                <p>
                  This 13-week graduate course is structured into 4 comprehensive modules, 
                  each building upon the previous to provide a complete understanding of 
                  Physical AI and Humanoid Robotics.
                </p>
              </div>
            </div>
            <div className="row">
              {modules.map((module, index) => (
                <ModuleCard
                  key={index}
                  title={module.title}
                  description={module.description}
                  chapters={module.chapters}
                  to={module.to}
                />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}