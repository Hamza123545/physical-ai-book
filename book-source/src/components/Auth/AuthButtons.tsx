/**
 * Auth Buttons Component
 *
 * Shows Login/Signup links or UserMenu based on auth state.
 * Uses AuthContext for state management.
 */

import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import UserMenu from './UserMenu';
import { useAuth } from '../../contexts/AuthContext';

const AuthButtons: React.FC = () => {
  const [mounted, setMounted] = useState(false);
  const { isAuthenticated, user } = useAuth();

  useEffect(() => {
    setMounted(true);
  }, []);

  // Don't render during SSR
  if (!mounted) {
    return null;
  }

  // Show user menu if authenticated
  if (isAuthenticated && user) {
    return (
      <div style={{ marginLeft: '1rem' }}>
        <UserMenu />
      </div>
    );
  }

  // Show login/signup links
  return (
    <div style={{ 
      display: 'flex', 
      gap: '0.5rem', 
      alignItems: 'center',
      marginLeft: '1rem'
    }}>
      <Link
        to="/login"
        style={{
          padding: '0.5rem 1rem',
          borderRadius: '6px',
          fontSize: '0.875rem',
          fontWeight: 500,
          textDecoration: 'none',
          color: 'var(--ifm-color-primary)',
          border: '1px solid var(--ifm-color-primary)',
        }}
      >
        Login
      </Link>
      <Link
        to="/signup"
        style={{
          padding: '0.5rem 1rem',
          borderRadius: '6px',
          fontSize: '0.875rem',
          fontWeight: 500,
          textDecoration: 'none',
          background: 'var(--ifm-color-primary)',
          color: 'white',
        }}
      >
        Sign Up
      </Link>
    </div>
  );
};

export default AuthButtons;
