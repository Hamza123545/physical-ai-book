/**
 * Authentication Context
 *
 * Global auth state management using React Context API.
 * Handles login, signup, logout, and token management.
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { signin, signup, getCurrentUser, type User, type AuthTokens } from '../services/authApi';
import { getAuthToken } from '../services/api';

// Re-export types from authApi
export type { User, AuthTokens } from '../services/authApi';

// Auth context interface
interface AuthContextType {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, full_name?: string) => Promise<void>;
  logout: () => void;
  clearError: () => void;
}

// Create context
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Provider props
interface AuthProviderProps {
  children: ReactNode;
}

// Auth Provider Component
export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Check for existing auth on mount
  useEffect(() => {
    const initAuth = async () => {
      const storedToken = getAuthToken();

      if (storedToken) {
        try {
          // Verify token by fetching user profile
          const userData = await getCurrentUser();
          setUser(userData);
          setToken(storedToken);
        } catch (err) {
          console.error('Failed to verify stored token:', err);
          localStorage.removeItem('authToken');
        }
      }

      setIsLoading(false);
    };

    initAuth();
  }, []);

  // Login function
  const login = async (email: string, password: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const data = await signin({ email, password });

      // Store token
      localStorage.setItem('authToken', data.tokens.access_token);

      // Update state
      setToken(data.tokens.access_token);
      setUser(data.user);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Login failed';
      setError(errorMessage);
      throw err; // Re-throw for component error handling
    } finally {
      setIsLoading(false);
    }
  };

  // Signup function
  const signupUser = async (email: string, password: string, full_name?: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const data = await signup({ email, password, full_name });

      // Store token
      localStorage.setItem('authToken', data.tokens.access_token);

      // Update state
      setToken(data.tokens.access_token);
      setUser(data.user);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Signup failed';
      setError(errorMessage);
      throw err; // Re-throw for component error handling
    } finally {
      setIsLoading(false);
    }
  };

  // Logout function
  const logout = () => {
    localStorage.removeItem('authToken');
    setToken(null);
    setUser(null);
    setError(null);
  };

  // Clear error
  const clearError = () => {
    setError(null);
  };

  const value: AuthContextType = {
    user,
    token,
    isAuthenticated: !!user && !!token,
    isLoading,
    error,
    login,
    signup: signupUser,
    logout,
    clearError,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
