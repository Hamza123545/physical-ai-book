/**
 * User API Service
 * 
 * Handles user background/profile management API calls
 */

import { apiRequest, handleApiResponse } from './api';

export interface UserBackground {
  id: string;
  user_id: string;
  software_experience: string;
  hardware_experience: string;
  robotics_experience: string;
  programming_languages?: string | null;
  learning_goals?: string | null;
  current_role: string;
  industry?: string | null;
  created_at: string;
  updated_at: string;
}

export interface UserBackgroundCreate {
  software_experience: string;
  hardware_experience: string;
  robotics_experience: string;
  programming_languages?: string;
  learning_goals?: string;
  current_role: string;
  industry?: string;
}

export interface UserBackgroundResponse {
  success: boolean;
  message: string;
  background: UserBackground;
}

/**
 * Get user background information
 */
export const getUserBackground = async (): Promise<UserBackground> => {
  const response = await apiRequest('/api/user/background', {
    method: 'GET',
  });
  
  return handleApiResponse<UserBackground>(response);
};

/**
 * Create or update user background
 */
export const submitUserBackground = async (
  data: UserBackgroundCreate
): Promise<UserBackgroundResponse> => {
  const response = await apiRequest('/api/user/background', {
    method: 'POST',
    body: JSON.stringify(data),
  });
  
  return handleApiResponse<UserBackgroundResponse>(response);
};

/**
 * Delete user background
 */
export const deleteUserBackground = async (): Promise<{ success: boolean; message: string }> => {
  const response = await apiRequest('/api/user/background', {
    method: 'DELETE',
  });
  
  return handleApiResponse<{ success: boolean; message: string }>(response);
};

