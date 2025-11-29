/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with LessonContent
 * to provide tabbed interface for Full Lesson and AI Summary views.
 *
 * The summary is read from global data (populated by docusaurus-summaries-plugin)
 * which scans for .summary.md files at build time.
 */

import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof ContentType>;

/**
 * DocItem/Content Theme Swizzle (Wrap)
 * 
 * Simplified version - just wraps the original content.
 * LessonContent component removed as it's not needed for hackathon.
 */
export default function ContentWrapper(props: Props): React.ReactElement {
  // Simply return the original content
  return <Content {...props} />;
}
