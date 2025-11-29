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
  // Use useBaseUrl for proper baseUrl handling in Docusaurus
  const imageUrl = useBaseUrl("/img/main.jpeg");
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroGradient} />
      <div className="container">
        <div className={styles.heroContent}>
          {/* Left side - Book Cover */}
          <div className={styles.heroImageContainer}>
            <img
              src={imageUrl}
              alt="Physical AI & Humanoid Robotics Textbook Cover"
              className={styles.heroBookCover}
              loading="eager"
              width="500"
              height="700"
            />
          </div>

          {/* Right side - Content */}
          <div className={styles.heroTextContent}>
            <div className={styles.heroLabel}>
              Hamza Swati - AI-Native Book Series
            </div>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </Heading>
            <p className={styles.heroSubtitle}>
              Building Intelligent Humanoid Robots with AI –{" "}
              <strong>Spec Driven Reusable Intelligence</strong>
            </p>

            <div className={styles.heroBadges}>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>✨</span>
                Open Source
              </span>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>🤝</span>
                Co-Learning with AI
              </span>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>🎯</span>
                Spec-Driven Development
              </span>
            </div>

            <div className={styles.heroButtons}>
              <Link
                className={clsx(
                  "button button--primary button--lg",
                  styles.ctaButton
                )}
                to="/docs/intro"
              >
                <span className={styles.buttonContent}>
                  <span className={styles.buttonText}>Start Learning</span>
                  <span className={styles.buttonIcon}>🤖</span>
                </span>
              </Link>
              <Link
                className={clsx(
                  "button button--outline button--lg",
                  styles.secondaryButton
                )}
                href="https://github.com/Hamza123545/physical-ai-book"
                target="_blank"
                rel="noopener noreferrer"
              >
                <span className={styles.buttonContent}>
                  <span className={styles.buttonText}>View on GitHub</span>
                  <span className={styles.buttonIcon}>⭐</span>
                </span>
              </Link>
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
        <div className={styles.spectrumHeader}>
          <div className={styles.spectrumLabel}>
            Understanding AI Development
          </div>
          <Heading as="h2" className={styles.spectrumTitle}>
            The AI Development Spectrum
          </Heading>
          <p className={styles.spectrumSubtitle}>
            Three distinct approaches to AI in robotics development. This book
            teaches you both AI-Driven and AI-Native development for building intelligent robots.
          </p>
        </div>

        <div className={styles.spectrumCards}>
          {/* AI Assisted */}
          <div className={styles.spectrumCard}>
            <div className={styles.spectrumCardHeader}>
              <div className={styles.spectrumIcon}>🛠️</div>
              <h3 className={styles.spectrumCardTitle}>AI Assisted</h3>
              <div className={styles.spectrumCardSubtitle}>AI as Helper</div>
            </div>
            <p className={styles.spectrumCardDescription}>
              AI enhances your productivity with code completion, debugging
              assistance, and documentation generation.
            </p>
            <ul className={styles.spectrumCardList}>
              <li>Code completion & suggestions</li>
              <li>Bug detection & debugging</li>
              <li>Documentation generation</li>
            </ul>
            <div className={styles.spectrumCardExample}>
              <strong>Example:</strong> Using GitHub Copilot to build ROS 2 nodes faster
            </div>
          </div>

          {/* AI Driven */}
          <div
            className={clsx(styles.spectrumCard, styles.spectrumCardHighlight)}
          >
            <div className={styles.spectrumBadge}>Focus of This Book</div>
            <div className={styles.spectrumCardHeader}>
              <div className={styles.spectrumIcon}>🚀</div>
              <h3 className={styles.spectrumCardTitle}>AI Driven</h3>
              <div className={styles.spectrumCardSubtitle}>
                AI as Co-Creator
              </div>
            </div>
            <p className={styles.spectrumCardDescription}>
              AI generates significant code from specifications. You act as
              architect, director, and reviewer.
            </p>
            <ul className={styles.spectrumCardList}>
              <li>Code generation from specs</li>
              <li>Automated testing & optimization</li>
              <li>Architecture from requirements</li>
            </ul>
            <div className={styles.spectrumCardExample}>
              <strong>Example:</strong> Writing a spec for humanoid navigation, AI
              generates complete Nav2 configuration and ROS 2 nodes
            </div>
          </div>

          {/* AI Native */}
          <div
            className={clsx(styles.spectrumCard, styles.spectrumCardHighlight)}
          >
            <div className={styles.spectrumBadge}>Focus of This Book</div>
            <div className={styles.spectrumCardHeader}>
              <div className={styles.spectrumIcon}>🤖</div>
              <h3 className={styles.spectrumCardTitle}>AI Native</h3>
              <div className={styles.spectrumCardSubtitle}>
                AI IS the Robot's Intelligence
              </div>
            </div>
            <p className={styles.spectrumCardDescription}>
              Robots architected around AI capabilities. Vision-Language-Action models
              and RL agents are core components.
            </p>
            <ul className={styles.spectrumCardList}>
              <li>Vision-Language-Action (VLA) models</li>
              <li>Reinforcement learning for locomotion</li>
              <li>LLM-based cognitive planning</li>
            </ul>
            <div className={styles.spectrumCardExample}>
              <strong>Example:</strong> Humanoid robot that understands voice commands
              and executes complex tasks autonomously
            </div>
          </div>
        </div>

        {/* Progression visualization */}
        <div className={styles.spectrumFlow}>
          <div className={styles.spectrumFlowTrack}>
            <div className={styles.spectrumFlowStep}>
              <div className={styles.flowStepCircle}>
                <span className={styles.flowStepNumber}>1</span>
              </div>
              <div className={styles.flowStepContent}>
                <div className={styles.flowStepTitle}>AI Assisted</div>
                <div className={styles.flowStepSubtitle}>Helper</div>
              </div>
            </div>

            <div className={styles.spectrumFlowLine}>
              <div className={styles.flowLineProgress}></div>
            </div>

            <div
              className={clsx(styles.spectrumFlowStep, styles.flowStepActive)}
            >
              <div className={styles.flowStepCircle}>
                <span className={styles.flowStepNumber}>2</span>
              </div>
              <div className={styles.flowStepContent}>
                <div className={styles.flowStepTitle}>AI Driven</div>
                <div className={styles.flowStepSubtitle}>Co-Creator</div>
              </div>
            </div>

            <div className={styles.spectrumFlowLine}>
              <div className={styles.flowLineProgress}></div>
            </div>

            <div
              className={clsx(styles.spectrumFlowStep, styles.flowStepActive)}
            >
              <div className={styles.flowStepCircle}>
                <span className={styles.flowStepNumber}>3</span>
              </div>
              <div className={styles.flowStepContent}>
                <div className={styles.flowStepTitle}>AI Native</div>
                <div className={styles.flowStepSubtitle}>Core System</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function Feature({
  title,
  description,
  icon,
  featured,
}: {
  title: string;
  description: string;
  icon: string;
  featured?: boolean;
}) {
  return (
    <div className={clsx(styles.feature, featured && styles.featureFeatured)}>
      {featured && <div className={styles.featureBadge}>Most Popular</div>}
      <div className={styles.featureIconWrapper}>
        <div className={styles.featureIcon}>{icon}</div>
      </div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
      <div className={styles.featureAccent} />
    </div>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <div className={styles.featuresLabel}>Core Pillars</div>
          <Heading as="h2" className={styles.featuresHeading}>
            What Makes This Book Different
          </Heading>
          <p className={styles.featuresSubheading}>
            A comprehensive, hands-on approach to building intelligent humanoid robots
            using AI-native development
          </p>
        </div>

        <div className={styles.featuresGrid}>
          <Feature
            icon="🤖"
            title="Co-Learning with AI"
            description="Learn alongside AI using the Three Roles Framework: AI as Teacher, Copilot, and Evaluator throughout your robotics journey."
            featured={true}
          />
          <Feature
            icon="🐍"
            title="Browser-Based Python"
            description="All exercises run in your browser via Pyodide. No installation required - learn robotics algorithms with instant feedback."
          />
          <Feature
            icon="📋"
            title="Spec-Driven Development"
            description="This textbook was built using SpecKit Plus. Learn the same AI-driven methodology used to create it."
          />
          <Feature
            icon="🦾"
            title="Complete Robotics Stack"
            description="Master ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, and Vision-Language-Action models from foundations to deployment."
          />
          <Feature
            icon="🏗️"
            title="Humanoid Robot Focus"
            description="Build bipedal locomotion, balance control, manipulation systems, and natural human-robot interaction."
          />
          <Feature
            icon="🚀"
            title="13-Week Learning Journey"
            description="7 chapters, 39 lessons with 100+ interactive exercises. Complete the Capstone: build an autonomous humanoid robot."
            featured={true}
          />
        </div>
      </div>
    </section>
  );
}

function MaturityLevelsSection() {
  return (
    <section className={styles.maturitySection}>
      <div className="container">
        <div className={styles.maturityHeader}>
          <div className={styles.maturityLabel}>Your AI Journey</div>
          <Heading as="h2" className={styles.maturityTitle}>
            Organizational AI Maturity Levels
          </Heading>
          <p className={styles.maturitySubtitle}>
            Where does your organization stand? Understanding these levels helps
            you chart your path forward.
          </p>
        </div>

        <div className={styles.maturityLevels}>
          {/* Level 1 */}
          <div className={styles.maturityLevel}>
            <div className={styles.maturityLevelNumber}>1</div>
            <div className={styles.maturityLevelHeader}>
              <div>
                <h3 className={styles.maturityLevelTitle}>AI Awareness</h3>
                <div className={styles.maturityLevelSubtitle}>
                  Experimenting
                </div>
              </div>
              <div className={styles.maturityLevelImpact}>
                10-20% productivity gains
              </div>
            </div>
            <p className={styles.maturityLevelDescription}>
              Individual developers experimenting with AI coding tools. Early AI
              Assisted Development.
            </p>
            <div className={styles.maturityLevelApproach}>
              <strong>Approach:</strong> AI Assisted (Individual)
            </div>
          </div>

          {/* Level 2 */}
          <div className={styles.maturityLevel}>
            <div className={styles.maturityLevelNumber}>2</div>
            <div className={styles.maturityLevelHeader}>
              <div>
                <h3 className={styles.maturityLevelTitle}>AI Adoption</h3>
                <div className={styles.maturityLevelSubtitle}>
                  Standardizing
                </div>
              </div>
              <div className={styles.maturityLevelImpact}>
                30-40% productivity boost
              </div>
            </div>
            <p className={styles.maturityLevelDescription}>
              Organization-wide adoption with governance. Established guidelines
              and security policies.
            </p>
            <div className={styles.maturityLevelApproach}>
              <strong>Approach:</strong> AI Assisted (Team)
            </div>
          </div>

          {/* Level 3 */}
          <div
            className={clsx(
              styles.maturityLevel,
              styles.maturityLevelHighlight
            )}
          >
            <div className={styles.maturityBadge}>BOOK FOCUS</div>
            <div className={styles.maturityLevelNumber}>3</div>
            <div className={styles.maturityLevelHeader}>
              <div>
                <h3 className={styles.maturityLevelTitle}>AI Integration</h3>
                <div className={styles.maturityLevelSubtitle}>
                  Transforming Workflows
                </div>
              </div>
              <div className={styles.maturityLevelImpact}>
                2-3x faster development
              </div>
            </div>
            <p className={styles.maturityLevelDescription}>
              AI-Driven Development practices. Specs become living
              documentation. Workflows redesigned around AI collaboration.
            </p>
            <div className={styles.maturityLevelApproach}>
              <strong>Approach:</strong> AI Driven (Workflow)
            </div>
          </div>

          {/* Level 4 */}
          <div
            className={clsx(
              styles.maturityLevel,
              styles.maturityLevelHighlight
            )}
          >
            <div className={styles.maturityBadge}>BOOK FOCUS</div>
            <div className={styles.maturityLevelNumber}>4</div>
            <div className={styles.maturityLevelHeader}>
              <div>
                <h3 className={styles.maturityLevelTitle}>
                  AI-Native Products
                </h3>
                <div className={styles.maturityLevelSubtitle}>
                  Building Intelligence
                </div>
              </div>
              <div className={styles.maturityLevelImpact}>
                New capabilities unlocked
              </div>
            </div>
            <p className={styles.maturityLevelDescription}>
              Products where AI/LLMs are core components. Agent orchestration,
              natural language interfaces, intelligent systems.
            </p>
            <div className={styles.maturityLevelApproach}>
              <strong>Approach:</strong> AI Native (Product)
            </div>
          </div>

          {/* Level 5 */}
          <div className={styles.maturityLevel}>
            <div className={styles.maturityLevelNumber}>5</div>
            <div className={styles.maturityLevelHeader}>
              <div>
                <h3 className={styles.maturityLevelTitle}>
                  AI-First Enterprise
                </h3>
                <div className={styles.maturityLevelSubtitle}>
                  Living in the Future
                </div>
              </div>
              <div className={styles.maturityLevelImpact}>10x productivity</div>
            </div>
            <p className={styles.maturityLevelDescription}>
              Entire organization AI-native. Custom models, self-improving
              systems, AI embedded in every aspect.
            </p>
            <div className={styles.maturityLevelApproach}>
              <strong>Approach:</strong> AI Native (Enterprise)
            </div>
          </div>
        </div>

        <div className={styles.maturityCTA}>
          <p className={styles.maturityCTAText}>
            <strong>This book prepares you for Levels 3-4:</strong> Master
            AI-Driven workflows and build AI-Native products
          </p>
        </div>
      </div>
    </section>
  );
}

function ParadigmShift() {
  return (
    <section className={styles.paradigmSection}>
      <div className="container">
        <div className={styles.paradigmContent}>
          <div className={styles.paradigmHeader}>
            <div className={styles.paradigmLabel}>The Great Shift</div>
            <Heading as="h2" className={styles.paradigmTitle}>
              From Automation to Intelligence
              <br />
              <span className={styles.paradigmTitleAccent}>
                From Coding to Co-Creating
              </span>
            </Heading>
            <p className={styles.paradigmSubtitle}>
              AI-native development is not about replacing developers—it's about
              amplifying intelligence. Learn to collaborate with reasoning
              entities that learn with you.
            </p>
          </div>

          <div className={styles.comparisonGrid}>
            {/* Traditional Card */}
            <div className={styles.comparisonCard}>
              <div className={styles.comparisonIconWrapper}>
                <div className={styles.comparisonIcon}>📚</div>
              </div>
              <div className={styles.comparisonLabel}>
                Traditional Development
              </div>
              <div className={styles.comparisonDescription}>
                The automation era
              </div>
              <ul className={styles.comparisonList}>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Instruction-Based
                  </span>
                  Tell computers exactly what to do with precise syntax
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Solo Coding
                  </span>
                  Developer writes every line manually
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Documentation as Afterthought
                  </span>
                  Specs are static contracts written post-facto
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Linear Learning
                  </span>
                  Learn syntax → Build simple projects → Slowly scale
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>Code-First</span>
                  Focus on implementation details from day one
                </li>
              </ul>
            </div>

            {/* VS Divider */}
            <div className={styles.comparisonDivider}>
              <div className={styles.comparisonVS}>VS</div>
              <div className={styles.comparisonArrow}>→</div>
            </div>

            {/* AI-Native Card */}
            <div
              className={clsx(
                styles.comparisonCard,
                styles.comparisonCardHighlight
              )}
            >
              <div className={styles.comparisonIconWrapper}>
                <div className={styles.comparisonIcon}>🤖</div>
              </div>
              <div className={styles.comparisonLabel}>AI-Native Way</div>
              <div className={styles.comparisonDescription}>
                The intelligence era
              </div>
              <ul className={styles.comparisonList}>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Intent-Based
                  </span>
                  Describe what you want; AI reasons how to build it
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Co-Learning Partnership
                  </span>
                  You and AI teach each other through iteration
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Specs as Living Blueprints
                  </span>
                  Specifications drive code, tests, and documentation
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Production-First Learning
                  </span>
                  Build real agentic systems from day one
                </li>
                <li>
                  <span className={styles.comparisonItemTitle}>
                    Architecture-First
                  </span>
                  Design intelligent collaborations, not just code
                </li>
              </ul>
            </div>
          </div>

          {/* Bottom CTA */}
          <div className={styles.paradigmCTA}>
            <div className={styles.paradigmCTAContent}>
              <div className={styles.paradigmCTAIcon}>🌱</div>
              <div className={styles.paradigmCTAText}>
                <h3 className={styles.paradigmCTATitle}>
                  Ready to Co-Learn with AI?
                </h3>
                <p className={styles.paradigmCTADescription}>
                  Join the revolution where coding becomes conversation and
                  software becomes alive
                </p>
              </div>
              <Link
                className={clsx(
                  "button button--primary button--lg",
                  styles.paradigmCTAButton
                )}
                to="/docs/intro"
              >
                Begin Your Robotics Journey
              </Link>
            </div>
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
      <MaturityLevelsSection />
      <ParadigmShift />
    </Layout>
  );
}
