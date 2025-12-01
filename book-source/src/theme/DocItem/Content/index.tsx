/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with PersonalizeButton
 * to enable content personalization for authenticated users.
 */

import React, { useMemo, useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import PersonalizeButton from '@/components/PersonalizeButton/PersonalizeButton';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { useAuth } from '@/contexts/AuthContext';
import styles from './styles.module.css';

type Props = WrapperProps<typeof ContentType>;

/**
 * DocItem/Content Theme Swizzle (Wrap)
 * 
 * Adds PersonalizeButton above the content for authenticated users.
 * When personalized, shows the personalized content in a section above the original.
 */
export default function ContentWrapper(props: Props): React.ReactElement {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [personalizationMetadata, setPersonalizationMetadata] = useState<any>(null);
  const [showPersonalized, setShowPersonalized] = useState(true);
  
  // Generate chapter ID from current URL path
  // Example: "/physical-ai-book/docs/chapter-01/intro" -> "chapter-01/intro"
  const chapterId = useMemo(() => {
    if (typeof window === 'undefined') return '';
    
    const path = location.pathname;
    const baseUrl = siteConfig.baseUrl || '';
    
    // Remove baseUrl and /docs/ prefix
    let docPath = path;
    if (baseUrl && path.startsWith(baseUrl)) {
      docPath = path.substring(baseUrl.length);
    }
    
    // Remove /docs/ prefix if present
    docPath = docPath.replace(/^\/docs\//, '').replace(/^docs\//, '');
    
    // Remove trailing slash
    docPath = docPath.replace(/\/$/, '');
    
    return docPath || '';
  }, [location.pathname, siteConfig.baseUrl]);
  
  // Handle personalized content
  const handlePersonalized = (content: string, metadata: any) => {
    setPersonalizedContent(content);
    setPersonalizationMetadata(metadata);
    setShowPersonalized(true);
  };

  const handleReset = () => {
    setPersonalizedContent(null);
    setPersonalizationMetadata(null);
    setShowPersonalized(true);
  };

  return (
    <div className={styles.contentWrapper}>
      {isAuthenticated && chapterId && (
        <div className={styles.personalizeSection}>
          {!personalizedContent ? (
            <PersonalizeButton
              chapterId={chapterId}
              onPersonalized={handlePersonalized}
            />
          ) : (
            <div className={styles.personalizedControls}>
              <div className={styles.statusBadge}>
                {personalizationMetadata?.cacheHit ? (
                  <span className={styles.cached}>⚡ From Cache</span>
                ) : (
                  <span className={styles.fresh}>✨ Freshly Personalized</span>
                )}
              </div>
              <div className={styles.toggleButtons}>
                <button
                  onClick={() => setShowPersonalized(true)}
                  className={`${styles.toggleButton} ${showPersonalized ? styles.active : ''}`}
                >
                  Personalized
                </button>
                <button
                  onClick={() => setShowPersonalized(false)}
                  className={`${styles.toggleButton} ${!showPersonalized ? styles.active : ''}`}
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
              {personalizationMetadata && showPersonalized && (
                <div className={styles.metadata}>
                  <div className={styles.metadataItem}>
                    <span className={styles.label}>Generation Time:</span>
                    <span className={styles.value}>
                      {personalizationMetadata.generation_time_ms
                        ? `${(personalizationMetadata.generation_time_ms / 1000).toFixed(2)}s`
                        : 'N/A'}
                    </span>
                  </div>
                  {personalizationMetadata.tokens_used && (
                    <div className={styles.metadataItem}>
                      <span className={styles.label}>Tokens Used:</span>
                      <span className={styles.value}>{personalizationMetadata.tokens_used.toLocaleString()}</span>
                    </div>
                  )}
                  {personalizationMetadata.model_used && (
                    <div className={styles.metadataItem}>
                      <span className={styles.label}>Model:</span>
                      <span className={styles.value}>{personalizationMetadata.model_used}</span>
                    </div>
                  )}
                </div>
              )}
            </div>
          )}
        </div>
      )}
      
      {personalizedContent && showPersonalized ? (
        <div className={styles.personalizedContent}>
          <ReactMarkdown remarkPlugins={[remarkGfm]}>
            {personalizedContent}
          </ReactMarkdown>
        </div>
      ) : (
        <Content {...props} />
      )}
    </div>
  );
}
