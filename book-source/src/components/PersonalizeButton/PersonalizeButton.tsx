import React, { useState } from 'react';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalized: (content: string, metadata: any) => void;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, onPersonalized }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    setIsLoading(true);
    setError(null);

    try {
      // Get JWT token from localStorage (assuming Better Auth stores it there)
      const token = localStorage.getItem('authToken');

      if (!token) {
        setError('Please sign in to personalize content');
        setIsLoading(false);
        return;
      }

      const response = await fetch('http://localhost:8000/api/content/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error?.message || 'Personalization failed');
      }

      const data = await response.json();

      if (data.success) {
        onPersonalized(data.personalized_content, {
          cacheHit: data.cache_hit,
          ...data.metadata,
        });
      } else {
        throw new Error('Personalization request was unsuccessful');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to personalize content');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.personalizeContainer}>
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        className={styles.personalizeButton}
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : (
          <>
            <span className={styles.icon}>✨</span>
            Personalize for Me
          </>
        )}
      </button>
      {error && (
        <div className={styles.error}>
          {error}
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;
