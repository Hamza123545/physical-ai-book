import React from 'react';
import styles from './styles.module.css';

export interface TryWithAIProps {
  id: string;
  title: string;
  role: 'Teacher' | 'Copilot' | 'Evaluator';
  scenario: string;
  yourTask: string;
  aiPromptTemplate: string;
  successCriteria: string[];
  reflectionQuestions: string[];
}

/**
 * TryWithAI component for AI co-learning exercises
 * Implements the AI Three Roles Framework (Teacher, Copilot, Evaluator)
 */
export default function TryWithAI({
  id,
  title,
  role,
  scenario,
  yourTask,
  aiPromptTemplate,
  successCriteria,
  reflectionQuestions,
}: TryWithAIProps) {
  const roleIcons = {
    Teacher: '👨‍🏫',
    Copilot: '🤝',
    Evaluator: '📊',
  };

  const roleDescriptions = {
    Teacher: 'AI explains concepts and provides guided learning',
    Copilot: 'AI collaborates with you to solve problems together',
    Evaluator: 'AI reviews and provides feedback on your work',
  };

  return (
    <div className={styles.tryWithAI} id={id}>
      <div className={styles.header}>
        <div className={styles.roleBadge} data-role={role.toLowerCase()}>
          <span className={styles.roleIcon}>{roleIcons[role]}</span>
          <span className={styles.roleName}>AI {role}</span>
        </div>
        <h3 className={styles.title}>{title}</h3>
      </div>

      <div className={styles.roleDescription}>
        {roleDescriptions[role]}
      </div>

      <div className={styles.section}>
        <h4>🎯 Scenario</h4>
        <p>{scenario}</p>
      </div>

      <div className={styles.section}>
        <h4>✋ Your Task (Try It First!)</h4>
        <p>{yourTask}</p>
      </div>

      <div className={styles.section}>
        <h4>💬 AI Prompt Template</h4>
        <div className={styles.promptBox}>
          <code>{aiPromptTemplate}</code>
        </div>
        <p className={styles.hint}>
          Copy this template and use it with ChatGPT, Claude, or another AI assistant.
          Replace [placeholders] with your actual code or context.
        </p>
      </div>

      <div className={styles.section}>
        <h4>✅ Success Criteria</h4>
        <ul className={styles.criteriaList}>
          {successCriteria.map((criterion, index) => (
            <li key={index}>{criterion}</li>
          ))}
        </ul>
      </div>

      <div className={styles.section}>
        <h4>🤔 Reflection Questions</h4>
        <ul className={styles.reflectionList}>
          {reflectionQuestions.map((question, index) => (
            <li key={index}>{question}</li>
          ))}
        </ul>
      </div>
    </div>
  );
}
