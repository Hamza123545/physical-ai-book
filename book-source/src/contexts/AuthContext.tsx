/**
 * Authentication Context
 *
 * Global auth state management using localStorage.
 * Works with FastAPI JWT backend.
 */

import React, { createContext, useContext, useState, useEffect, ReactNode, useCallback } from 'react';

// User type matching FastAPI backend
export interface User {
  id: string;
  email: string;
  full_name?: string | null;
  is_active?: boolean;
  is_verified?: boolean;
}

// Auth context interface
interface AuthContextType {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, name?: string) => Promise<void>;
  logout: () => void;
  clearError: () => void;
}

// Create context
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Get API base URL
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:8000';
  if (window.location.hostname === 'localhost') return 'http://localhost:8000';
  return 'https://physical-ai-backend-9lxv.onrender.com';
};

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

  // Load auth state from localStorage on mount
  useEffect(() => {
    const storedToken = localStorage.getItem('authToken');
    const storedUser = localStorage.getItem('user');
    
    if (storedToken && storedUser) {
      try {
        setToken(storedToken);
        setUser(JSON.parse(storedUser));
      } catch (e) {
        console.error('Failed to parse stored user:', e);
        localStorage.removeItem('authToken');
        localStorage.removeItem('user');
      }
    }
    setIsLoading(false);
  }, []);

  // Sync auth state across tabs
  useEffect(() => {
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'authToken' || e.key === 'user') {
        const storedToken = localStorage.getItem('authToken');
        const storedUser = localStorage.getItem('user');
        
        if (storedToken && storedUser) {
          try {
            setToken(storedToken);
            setUser(JSON.parse(storedUser));
          } catch (e) {
            setToken(null);
            setUser(null);
          }
        } else {
          setToken(null);
          setUser(null);
        }
      }
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  // Sync auth state across React trees in same tab
  useEffect(() => {
    const handleAuthStateChange = (e: CustomEvent<{ user: User | null; token: string | null }>) => {
      setUser(e.detail.user);
      setToken(e.detail.token);
    };

    window.addEventListener('authStateChange', handleAuthStateChange as EventListener);
    return () => window.removeEventListener('authStateChange', handleAuthStateChange as EventListener);
  }, []);

  // Login function
  const login = useCallback(async (email: string, password: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        const errorMsg = data.detail?.error?.message || data.detail || 'Login failed';
        throw new Error(errorMsg);
      }

      // Store token and user
      const accessToken = data.tokens?.access_token;
      const userData = data.user;

      if (accessToken) {
        localStorage.setItem('authToken', accessToken);
        setToken(accessToken);
      }
      if (userData) {
        localStorage.setItem('user', JSON.stringify(userData));
        setUser(userData);
      }

      // Dispatch event for other React trees
      window.dispatchEvent(new CustomEvent('authStateChange', {
        detail: { user: userData, token: accessToken }
      }));

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Login failed';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  // Signup function
  const signup = useCallback(async (email: string, password: string, name?: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password, full_name: name }),
      });

      const data = await response.json();

      if (!response.ok) {
        const errorMsg = data.detail?.error?.message || data.detail || 'Signup failed';
        throw new Error(errorMsg);
      }

      // Store token and user
      const accessToken = data.tokens?.access_token;
      const userData = data.user;

      if (accessToken) {
        localStorage.setItem('authToken', accessToken);
        setToken(accessToken);
      }
      if (userData) {
        localStorage.setItem('user', JSON.stringify(userData));
        setUser(userData);
      }

      // Dispatch event for other React trees
      window.dispatchEvent(new CustomEvent('authStateChange', {
        detail: { user: userData, token: accessToken }
      }));

    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Signup failed';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  // Logout function
  const logout = useCallback(() => {
    localStorage.removeItem('authToken');
    localStorage.removeItem('user');
    setToken(null);
    setUser(null);
    setError(null);

    // Dispatch event for other React trees
    window.dispatchEvent(new CustomEvent('authStateChange', {
      detail: { user: null, token: null }
    }));
  }, []);

  // Clear error
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  const value: AuthContextType = {
    user,
    token,
    isAuthenticated: !!user && !!token,
    isLoading,
    error,
    login,
    signup,
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
