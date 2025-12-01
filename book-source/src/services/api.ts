/**
 * Centralized API Configuration Service
 * 
 * Provides consistent API base URL and helper functions for all API calls
 */

// Get API base URL based on environment
export const getApiBaseUrl = (): string => {
  if (typeof window === 'undefined') {
    // Server-side rendering - default to localhost
    return 'http://localhost:8000';
  }
  
  // Check if we're in development (localhost)
  if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }
  
  // Production: Use Hugging Face Spaces backend URL
  // Note: In Docusaurus, we can't use process.env in client-side code
  // Environment variables should be set at build time via docusaurus.config.ts
  return 'https://hamza057-aibook.hf.space';
};

export const API_BASE_URL = getApiBaseUrl();

/**
 * Get authentication token from localStorage
 */
export const getAuthToken = (): string | null => {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem('authToken');
};

/**
 * Get headers with authentication token
 */
export const getAuthHeaders = (): HeadersInit => {
  const token = getAuthToken();
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };
  
  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }
  
  return headers;
};

/**
 * Make authenticated API request
 */
export const apiRequest = async (
  endpoint: string,
  options: RequestInit = {}
): Promise<Response> => {
  const url = `${API_BASE_URL}${endpoint}`;
  const headers = getAuthHeaders();
  
  // Merge custom headers with auth headers
  const mergedHeaders = {
    ...headers,
    ...(options.headers || {}),
  };
  
  // Log request for debugging (only in development)
  if (process.env.NODE_ENV === 'development') {
    console.log('API Request:', {
      url,
      method: options.method || 'GET',
      hasAuth: !!headers['Authorization'],
    });
  }
  
  const response = await fetch(url, {
    ...options,
    headers: mergedHeaders,
  });
  
  // Log response status for debugging
  if (process.env.NODE_ENV === 'development') {
    console.log('API Response:', {
      url,
      status: response.status,
      statusText: response.statusText,
    });
  }
  
  return response;
};

/**
 * Handle API response and extract JSON or throw error
 */
export const handleApiResponse = async <T>(response: Response): Promise<T> => {
  if (!response.ok) {
    const errorData = await response.json().catch(() => ({
      error: {
        message: `API error: ${response.status} ${response.statusText}`,
      },
    }));
    throw new Error(errorData.error?.message || 'API request failed');
  }
  
  return response.json();
};

