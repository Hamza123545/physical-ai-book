import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import useBaseUrl from "@docusaurus/useBaseUrl";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const imageUrl = useBaseUrl("/img/main.jpeg");
  
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGradient1} />
        <div className={styles.heroGradient2} />
        <div className={styles.heroGradient3} />
      </div>
      
      <div className="container">
        <div className={styles.heroContent}>
          {/* Left Side - Content */}
          <div className={styles.heroLeft}>
            <div className={styles.heroLabel}>
              <span className={styles.labelDot}></span>
              AI-Native Robotics Learning Platform
            </div>
            
            <Heading as="h1" className={styles.heroTitle}>
              Master Humanoid Robotics
              <br />
              <span className={styles.heroTitleAccent}>with AI Co-Learning</span>
            </Heading>
            
            <p className={styles.heroSubtitle}>
              Build intelligent robots using <strong>Spec-Driven Development</strong> and 
              learn alongside AI. From ROS 2 to Vision-Language-Action models—everything you need in one place.
            </p>
            
            <div className={styles.heroStats}>
              <div className={styles.statBox}>
                <div className={styles.statNumber}>7</div>
                <div className={styles.statLabel}>Chapters</div>
              </div>
              <div className={styles.statBox}>
                <div className={styles.statNumber}>39</div>
                <div className={styles.statLabel}>Lessons</div>
              </div>
              <div className={styles.statBox}>
                <div className={styles.statNumber}>100+</div>
                <div className={styles.statLabel}>Exercises</div>
              </div>
            </div>

            <div className={styles.heroButtons}>
              <Link
                className={clsx("button button--primary button--lg", styles.primaryBtn)}
                to="/docs/intro"
              >
                <span>Start Learning Now</span>
                <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
                  <path d="M7.5 15L12.5 10L7.5 5" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </Link>
              <Link
                className={clsx("button button--outline button--lg", styles.secondaryBtn)}
                href="https://github.com/Hamza123545/physical-ai-book"
                target="_blank"
                rel="noopener noreferrer"
              >
                <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
                  <path d="M10 2C5.58 2 2 5.58 2 10C2 13.42 4.16 16.34 7.25 17.75C7.75 17.84 7.92 17.5 7.92 17.21C7.92 16.96 7.91 16.13 7.91 15.25C5.5 15.75 4.84 14.67 4.67 14.13C4.58 13.84 4.08 12.84 3.58 12.59C3.17 12.38 2.75 11.84 3.58 11.84C4.34 11.84 4.84 12.5 5.08 12.84C5.5 13.59 6.34 13.34 7.92 12.84C8 12.25 8.25 11.84 8.5 11.59C6.17 11.25 3.75 10.25 3.75 7.34C3.75 6.5 4.08 5.75 4.67 5.17C4.58 4.92 4.25 4.17 4.75 3.13C4.75 3.13 5.34 2.92 7.92 4.17C8.67 3.96 9.5 3.84 10.34 3.84C11.17 3.84 12 3.96 12.75 4.17C15.34 2.92 15.92 3.13 15.92 3.13C16.42 4.17 16.08 4.92 16 5.17C16.58 5.75 16.92 6.5 16.92 7.34C16.92 10.25 14.5 11.25 12.17 11.59C12.5 11.92 12.75 12.5 12.75 13.34C12.75 14.5 12.75 15.42 12.75 17.21C12.75 17.5 12.92 17.84 13.42 17.75C16.5 16.34 18.67 13.42 18.67 10C18.67 5.58 15.08 2 10.67 2H10Z" fill="currentColor"/>
                </svg>
                <span>View on GitHub</span>
              </Link>
            </div>
          </div>

          {/* Right Side - Book Cover */}
          <div className={styles.heroRight}>
            <div className={styles.bookContainer}>
              <div className={styles.bookGlow}></div>
              <img
                src={imageUrl}
                alt="Physical AI & Humanoid Robotics Textbook"
                className={styles.bookImage}
                loading="eager"
              />
              <div className={styles.bookBadges}>
                <div className={styles.bookBadge}>
                  <span className={styles.badgeIcon}>✨</span>
                  <span>Open Source</span>
                </div>
                <div className={styles.bookBadge}>
                  <span className={styles.badgeIcon}>🤖</span>
                  <span>AI-Powered</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function AISpectrumSection() {
  return (
    <section className={styles.spectrumSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionBadge}>The AI Spectrum</div>
          <Heading as="h2" className={styles.sectionTitle}>
            Three Ways AI Transforms Robotics Development
          </Heading>
          <p className={styles.sectionDescription}>
            Understand how AI can assist, drive, or become the core of your robotics projects
          </p>
        </div>

        <div className={styles.spectrumGrid}>
          <div className={styles.spectrumCard}>
            <div className={styles.cardHeader}>
              <div className={styles.cardNumber}>01</div>
              <div className={styles.cardIcon}>🛠️</div>
            </div>
            <h3 className={styles.cardTitle}>AI Assisted</h3>
            <p className={styles.cardSubtitle}>AI as Helper</p>
            <p className={styles.cardText}>
              Boost productivity with AI-powered code completion, debugging assistance, and documentation.
            </p>
            <ul className={styles.cardFeatures}>
              <li>Code completion & suggestions</li>
              <li>Bug detection & debugging</li>
              <li>Documentation generation</li>
            </ul>
          </div>

          <div className={clsx(styles.spectrumCard, styles.spectrumCardPrimary)}>
            <div className={styles.primaryBadge}>Book Focus</div>
            <div className={styles.cardHeader}>
              <div className={styles.cardNumber}>02</div>
              <div className={styles.cardIcon}>🚀</div>
            </div>
            <h3 className={styles.cardTitle}>AI Driven</h3>
            <p className={styles.cardSubtitle}>AI as Co-Creator</p>
            <p className={styles.cardText}>
              AI generates code from specifications. You architect, direct, and review—AI implements.
            </p>
            <ul className={styles.cardFeatures}>
              <li>Code generation from specs</li>
              <li>Automated testing & optimization</li>
              <li>Architecture from requirements</li>
            </ul>
          </div>

          <div className={clsx(styles.spectrumCard, styles.spectrumCardPrimary)}>
            <div className={styles.primaryBadge}>Book Focus</div>
            <div className={styles.cardHeader}>
              <div className={styles.cardNumber}>03</div>
              <div className={styles.cardIcon}>🤖</div>
            </div>
            <h3 className={styles.cardTitle}>AI Native</h3>
            <p className={styles.cardSubtitle}>AI IS the Intelligence</p>
            <p className={styles.cardText}>
              Robots built around AI capabilities. Vision-Language-Action models and RL agents are core components.
            </p>
            <ul className={styles.cardFeatures}>
              <li>Vision-Language-Action (VLA) models</li>
              <li>Reinforcement learning for locomotion</li>
              <li>LLM-based cognitive planning</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionBadge}>Why This Book?</div>
          <Heading as="h2" className={styles.sectionTitle}>
            Everything You Need to Build Intelligent Robots
          </Heading>
        </div>

        <div className={styles.featuresGrid}>
          <div className={clsx(styles.featureCard, styles.featureCardHighlight)}>
            <div className={styles.featureHighlightBadge}>Most Popular</div>
            <div className={styles.featureIcon}>🤖</div>
            <h3 className={styles.featureTitle}>Co-Learning with AI</h3>
            <p className={styles.featureText}>
              Learn alongside AI using the Three Roles Framework: AI as Teacher, Copilot, and Evaluator. 
              Experience a new way of learning where AI adapts to your pace and style.
            </p>
          </div>

          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🐍</div>
            <h3 className={styles.featureTitle}>Browser-Based Python</h3>
            <p className={styles.featureText}>
              All exercises run in your browser via Pyodide. No installation required—learn robotics algorithms with instant feedback.
            </p>
          </div>

          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>📋</div>
            <h3 className={styles.featureTitle}>Spec-Driven Development</h3>
            <p className={styles.featureText}>
              This textbook was built using SpecKit Plus. Learn the same AI-driven methodology used to create it.
            </p>
          </div>

          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🦾</div>
            <h3 className={styles.featureTitle}>Complete Robotics Stack</h3>
            <p className={styles.featureText}>
              Master ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, and Vision-Language-Action models from foundations to deployment.
            </p>
          </div>

          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🏗️</div>
            <h3 className={styles.featureTitle}>Humanoid Robot Focus</h3>
            <p className={styles.featureText}>
              Build bipedal locomotion, balance control, manipulation systems, and natural human-robot interaction.
            </p>
          </div>

          <div className={clsx(styles.featureCard, styles.featureCardHighlight)}>
            <div className={styles.featureHighlightBadge}>Capstone Project</div>
            <div className={styles.featureIcon}>🚀</div>
            <h3 className={styles.featureTitle}>13-Week Learning Journey</h3>
            <p className={styles.featureText}>
              7 chapters, 39 lessons with 100+ interactive exercises. Complete the Capstone: build an autonomous humanoid robot from scratch.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.ctaBackground}></div>
      <div className="container">
        <div className={styles.ctaContent}>
          <div className={styles.ctaIcon}>🎓</div>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build the Future?
          </Heading>
          <p className={styles.ctaDescription}>
            Join thousands of learners mastering AI-native robotics development. 
            Start your journey today and build intelligent robots that change the world.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx("button button--primary button--lg", styles.ctaPrimaryBtn)}
              to="/docs/intro"
            >
              Begin Learning Journey
            </Link>
            <Link
              className={clsx("button button--outline button--lg", styles.ctaSecondaryBtn)}
              to="/profile"
            >
              Set Your Learning Profile
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Building Intelligent Humanoid Robots with AI – Spec Driven Reusable Intelligence. Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models."
    >
      <HomepageHeader />
      <AISpectrumSection />
      <FeaturesSection />
      <CTASection />
    </Layout>
  );
}
