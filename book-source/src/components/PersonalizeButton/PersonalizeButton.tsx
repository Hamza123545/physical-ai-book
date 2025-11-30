import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { personalizeContent } from '../../services/personalizationApi';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalized: (content: string, metadata: any) => void;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, onPersonalized }) => {
  const { isAuthenticated } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      setError('Please sign in to personalize content');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const data = await personalizeContent(chapterId);

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
