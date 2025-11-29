import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export interface Prerequisite {
  lessonId: string;
  title: string;
  link: string;
}

export interface PrerequisitesProps {
  prerequisites: Prerequisite[];
  showCompletionStatus?: boolean;
}

/**
 * Prerequisites component displays a checklist of prerequisite lessons
 * with optional completion tracking from localStorage
 */
export default function Prerequisites({
  prerequisites,
  showCompletionStatus = true,
}: PrerequisitesProps) {
  const [completedLessons, setCompletedLessons] = useState<Set<string>>(new Set());

  useEffect(() => {
    if (showCompletionStatus && typeof window !== 'undefined') {
      try {
        const progressData = localStorage.getItem('studentProgress');
        if (progressData) {
          const progress = JSON.parse(progressData);
          const completed = new Set<string>(
            Object.entries(progress.lessons || {})
              .filter(([_, lessonProgress]: [string, any]) => lessonProgress.completed)
              .map(([lessonId, _]) => lessonId)
          );
          setCompletedLessons(completed);
        }
      } catch (error) {
        console.error('Error loading progress data:', error);
      }
    }
  }, [showCompletionStatus]);

  if (prerequisites.length === 0) {
    return null;
  }

  return (
    <div className={styles.prerequisites}>
      <div className={styles.header}>
        <h3 className={styles.title}>📚 Prerequisites</h3>
        <p className={styles.description}>
          Before starting this lesson, make sure you've completed:
        </p>
      </div>

      <ul className={styles.prerequisitesList}>
        {prerequisites.map((prereq, index) => {
          const isCompleted = completedLessons.has(prereq.lessonId);

          return (
            <li key={index} className={styles.prerequisiteItem}>
              {showCompletionStatus && (
                <span
                  className={styles.checkbox}
                  data-completed={isCompleted}
                  aria-label={isCompleted ? 'Completed' : 'Not completed'}
                >
                  {isCompleted ? '✅' : '⬜'}
                </span>
              )}
              <Link to={prereq.link} className={styles.prerequisiteLink}>
                {prereq.title}
              </Link>
            </li>
          );
        })}
      </ul>

      {showCompletionStatus && (
        <div className={styles.footer}>
          <p className={styles.hint}>
            💡 <strong>Tip:</strong> Complete prerequisites are marked with ✅.
            Your progress is saved automatically as you complete exercises.
          </p>
        </div>
      )}
    </div>
  );
}
