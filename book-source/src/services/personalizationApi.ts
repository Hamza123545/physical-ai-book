/**
 * Personalization API Service
 * 
 * Handles content personalization API calls
 */

import { apiRequest, handleApiResponse } from './api';

export interface PersonalizeRequest {
  chapter_id: string;
  user_id?: string;
}

export interface PersonalizeResponse {
  success: boolean;
  chapter_id: string;
  personalized_content: string;
  cache_hit: boolean;
  metadata?: {
    model_used?: string;
    tokens_used?: number;
    generation_time_ms?: number;
    user_id?: string;
    cached_at?: string;
  };
}

/**
 * Personalize chapter content for the current user
 */
export const personalizeContent = async (
  chapterId: string
): Promise<PersonalizeResponse> => {
  const response = await apiRequest('/api/content/personalize', {
    method: 'POST',
    body: JSON.stringify({
      chapter_id: chapterId,
    }),
  });
  
  return handleApiResponse<PersonalizeResponse>(response);
};

