import React from 'react';
import styles from './styles.module.css';

export interface LearningObjective {
  text: string;
  blooms_level: 'Remember' | 'Understand' | 'Apply' | 'Analyze' | 'Evaluate' | 'Create';
  assessment_method: string;
}

export interface LearningObjectivesProps {
  objectives: LearningObjective[];
  cefr_level: string;
}

/**
 * LearningObjectives component displays lesson objectives with CEFR and Bloom's taxonomy alignment
 */
export default function LearningObjectives({
  objectives,
  cefr_level,
}: LearningObjectivesProps) {
  const bloomsColors = {
    Remember: '#90caf9',
    Understand: '#81c784',
    Apply: '#ffb74d',
    Analyze: '#ba68c8',
    Evaluate: '#e57373',
    Create: '#ff8a65',
  };

  const bloomsIcons = {
    Remember: '📝',
    Understand: '💡',
    Apply: '⚙️',
    Analyze: '🔍',
    Evaluate: '⚖️',
    Create: '🎨',
  };

  return (
    <div className={styles.learningObjectives}>
      <div className={styles.header}>
        <h3 className={styles.title}>🎯 Learning Objectives</h3>
        <div className={styles.cefrBadge}>
          <span className={styles.cefrLabel}>CEFR Level:</span>
          <span className={styles.cefrValue}>{cefr_level}</span>
        </div>
      </div>

      <div className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <div key={index} className={styles.objective}>
            <div className={styles.objectiveHeader}>
              <span
                className={styles.bloomsBadge}
                style={{ backgroundColor: bloomsColors[objective.blooms_level] }}
              >
                <span className={styles.bloomsIcon}>
                  {bloomsIcons[objective.blooms_level]}
                </span>
                <span className={styles.bloomsLevel}>
                  {objective.blooms_level}
                </span>
              </span>
            </div>
            <div className={styles.objectiveText}>{objective.text}</div>
            <div className={styles.assessmentMethod}>
              <span className={styles.assessmentLabel}>Assessment:</span>{' '}
              {objective.assessment_method}
            </div>
          </div>
        ))}
      </div>

      <div className={styles.footer}>
        <p className={styles.cefrDescription}>
          <strong>CEFR {cefr_level}:</strong> This lesson is designed for{' '}
          {cefr_level.startsWith('A') && 'beginner-level'}
          {cefr_level.startsWith('B') && 'intermediate-level'}
          {cefr_level.startsWith('C') && 'advanced-level'}{' '}
          learners with appropriate complexity and prerequisites.
        </p>
      </div>
    </div>
  );
}
