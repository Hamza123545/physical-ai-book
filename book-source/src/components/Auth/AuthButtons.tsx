/**
 * Auth Buttons Component
 *
 * Shows Login/Signup buttons when not authenticated,
 * or UserMenu when authenticated.
 * Can be used in navbar or anywhere in the app.
 */

import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import LoginModal from './LoginModal';
import SignupModal from './SignupModal';
import UserMenu from './UserMenu';
import styles from './Auth.module.css';

const AuthButtons: React.FC = () => {
  const { isAuthenticated } = useAuth();
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);

  // If authenticated, show user menu
  if (isAuthenticated) {
    return <UserMenu />;
  }

  // If not authenticated, show login/signup buttons
  return (
    <>
      <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
        <button
          onClick={() => setShowLogin(true)}
          className={`${styles.authButton} ${styles.authButtonSecondary}`}
        >
          Login
        </button>
        <button
          onClick={() => setShowSignup(true)}
          className={`${styles.authButton} ${styles.authButtonPrimary}`}
        >
          Sign Up
        </button>
      </div>

      <LoginModal
        isOpen={showLogin}
        onClose={() => setShowLogin(false)}
        onSwitchToSignup={() => {
          setShowLogin(false);
          setShowSignup(true);
        }}
      />

      <SignupModal
        isOpen={showSignup}
        onClose={() => setShowSignup(false)}
        onSwitchToLogin={() => {
          setShowSignup(false);
          setShowLogin(true);
        }}
      />
    </>
  );
};

export default AuthButtons;

