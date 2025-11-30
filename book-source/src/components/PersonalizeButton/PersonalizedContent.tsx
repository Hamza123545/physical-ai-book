import React, { useState } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import PersonalizeButton from './PersonalizeButton';
import styles from './PersonalizedContent.module.css';

interface PersonalizedContentProps {
  chapterId: string;
  originalContent: string;
}

interface PersonalizationMetadata {
  cacheHit?: boolean;
  model_used?: string;
  tokens_used?: number;
  generation_time_ms?: number;
  user_id?: string;
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({ chapterId, originalContent }) => {
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [metadata, setMetadata] = useState<PersonalizationMetadata | null>(null);
  const [showOriginal, setShowOriginal] = useState(false);

  const handlePersonalized = (content: string, meta: PersonalizationMetadata) => {
    setPersonalizedContent(content);
    setMetadata(meta);
  };

  const handleReset = () => {
    setPersonalizedContent(null);
    setMetadata(null);
    setShowOriginal(false);
  };

  const displayContent = showOriginal ? originalContent : (personalizedContent || originalContent);
  const isPersonalized = personalizedContent !== null;

  return (
    <div className={styles.container}>
      {!isPersonalized && (
        <PersonalizeButton
          chapterId={chapterId}
          onPersonalized={handlePersonalized}
        />
      )}

      {isPersonalized && (
        <div className={styles.controls}>
          <div className={styles.statusBadge}>
            {metadata?.cacheHit ? (
              <span className={styles.cached}>⚡ From Cache</span>
            ) : (
              <span className={styles.fresh}>✨ Freshly Personalized</span>
            )}
          </div>

          <div className={styles.toggleButtons}>
            <button
              onClick={() => setShowOriginal(false)}
              className={`${styles.toggleButton} ${!showOriginal ? styles.active : ''}`}
            >
              Personalized
            </button>
            <button
              onClick={() => setShowOriginal(true)}
              className={`${styles.toggleButton} ${showOriginal ? styles.active : ''}`}
            >
              Original
            </button>
            <button
              onClick={handleReset}
              className={styles.resetButton}
            >
              Reset
            </button>
          </div>
        </div>
      )}

      {metadata && !showOriginal && (
        <div className={styles.metadata}>
          <div className={styles.metadataItem}>
            <span className={styles.label}>Generation Time:</span>
            <span className={styles.value}>
              {metadata.generation_time_ms
                ? `${(metadata.generation_time_ms / 1000).toFixed(2)}s`
                : 'N/A'}
            </span>
          </div>
          {metadata.tokens_used && (
            <div className={styles.metadataItem}>
              <span className={styles.label}>Tokens Used:</span>
              <span className={styles.value}>{metadata.tokens_used.toLocaleString()}</span>
            </div>
          )}
          {metadata.model_used && (
            <div className={styles.metadataItem}>
              <span className={styles.label}>Model:</span>
              <span className={styles.value}>{metadata.model_used}</span>
            </div>
          )}
        </div>
      )}

      <div className={styles.content}>
        <ReactMarkdown remarkPlugins={[remarkGfm]}>
          {displayContent}
        </ReactMarkdown>
      </div>
    </div>
  );
};

export default PersonalizedContent;
