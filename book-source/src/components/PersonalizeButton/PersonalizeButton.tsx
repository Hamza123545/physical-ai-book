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
      console.log('Personalizing chapter:', chapterId);
      const data = await personalizeContent(chapterId);
      console.log('Personalization response:', data);

      if (data.success) {
        onPersonalized(data.personalized_content, {
          cacheHit: data.cache_hit,
          ...data.metadata,
        });
      } else {
        throw new Error('Personalization request was unsuccessful');
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to personalize content';
      console.error('Personalization error:', err);
      setError(errorMessage);
      
      // Show more detailed error for debugging
      if (errorMessage.includes('404')) {
        setError('API endpoint not found. Please check if the backend is running.');
      } else if (errorMessage.includes('401')) {
        setError('Authentication failed. Please sign in again.');
      } else if (errorMessage.includes('403')) {
        setError('Access denied. Please check your permissions.');
      }
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
