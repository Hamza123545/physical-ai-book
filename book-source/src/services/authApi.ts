/**
 * Authentication API Service
 * 
 * Handles all authentication-related API calls
 */

import { apiRequest, handleApiResponse } from './api';

export interface User {
  id: string;
  email: string;
  full_name: string | null;
  is_active: boolean;
  is_verified: boolean;
  created_at: string;
  updated_at: string;
}

export interface AuthTokens {
  access_token: string;
  refresh_token?: string;
  token_type: string;
  expires_in: number;
}

export interface SignupRequest {
  email: string;
  password: string;
  full_name?: string;
}

export interface SigninRequest {
  email: string;
  password: string;
}

export interface SignupResponse {
  success: boolean;
  message: string;
  user: User;
  tokens: AuthTokens;
}

export interface SigninResponse {
  success: boolean;
  message: string;
  user: User;
  tokens: AuthTokens;
}

/**
 * Sign up a new user
 */
export const signup = async (data: SignupRequest): Promise<SignupResponse> => {
  const response = await apiRequest('/api/auth/signup', {
    method: 'POST',
    body: JSON.stringify(data),
  });
  
  return handleApiResponse<SignupResponse>(response);
};

/**
 * Sign in an existing user
 */
export const signin = async (data: SigninRequest): Promise<SigninResponse> => {
  const response = await apiRequest('/api/auth/signin', {
    method: 'POST',
    body: JSON.stringify(data),
  });
  
  return handleApiResponse<SigninResponse>(response);
};

/**
 * Get current authenticated user
 */
export const getCurrentUser = async (): Promise<User> => {
  const response = await apiRequest('/api/auth/me', {
    method: 'GET',
  });
  
  return handleApiResponse<User>(response);
};

