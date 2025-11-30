/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with PersonalizeButton
 * to enable content personalization for authenticated users.
 */

import React, { useMemo } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import PersonalizeButton from '@/components/PersonalizeButton/PersonalizeButton';
import { useAuth } from '@/contexts/AuthContext';
import styles from './styles.module.css';

type Props = WrapperProps<typeof ContentType>;

/**
 * DocItem/Content Theme Swizzle (Wrap)
 * 
 * Adds PersonalizeButton above the content for authenticated users.
 */
export default function ContentWrapper(props: Props): React.ReactElement {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  
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
    // For now, we'll show a notification
    // In future, we can replace the content dynamically
    console.log('Content personalized:', { content, metadata });
    // You can implement content replacement logic here
    alert('Content personalized! Check console for details.');
  };

  return (
    <div className={styles.contentWrapper}>
      {isAuthenticated && chapterId && (
        <div className={styles.personalizeSection}>
          <PersonalizeButton
            chapterId={chapterId}
            onPersonalized={handlePersonalized}
          />
        </div>
      )}
      <Content {...props} />
    </div>
  );
}
