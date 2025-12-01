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
  try {
    const response = await apiRequest('/api/content/personalize', {
      method: 'POST',
      body: JSON.stringify({
        chapter_id: chapterId,
      }),
    });
    
    // Log response for debugging
    console.log('Personalize API response status:', response.status);
    
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({
        error: {
          message: `API error: ${response.status} ${response.statusText}`,
        },
      }));
      console.error('Personalize API error:', errorData);
      throw new Error(errorData.error?.message || `Failed to personalize: ${response.status} ${response.statusText}`);
    }
    
    return handleApiResponse<PersonalizeResponse>(response);
  } catch (error) {
    console.error('Personalize API request failed:', error);
    throw error;
  }
};

