/**
 * Signup Modal Component
 *
 * Modal dialog for user signup with password requirements validation.
 * Password must have: 8+ chars, 1 uppercase, 1 lowercase, 1 digit.
 */

import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './Auth.module.css';

interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToLogin: () => void;
}

const SignupModal: React.FC<SignupModalProps> = ({ isOpen, onClose, onSwitchToLogin }) => {
  const { signup, clearError } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [fullName, setFullName] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [passwordErrors, setPasswordErrors] = useState<string[]>([]);

  // Validate password requirements
  const validatePassword = (pwd: string): string[] => {
    const errors: string[] = [];

    if (pwd.length < 8) {
      errors.push('At least 8 characters');
    }
    if (!/[A-Z]/.test(pwd)) {
      errors.push('One uppercase letter');
    }
    if (!/[a-z]/.test(pwd)) {
      errors.push('One lowercase letter');
    }
    if (!/\d/.test(pwd)) {
      errors.push('One digit');
    }

    return errors;
  };

  // Handle password change
  const handlePasswordChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newPassword = e.target.value;
    setPassword(newPassword);
    setPasswordErrors(validatePassword(newPassword));
  };

  // Handle modal close
  const handleClose = () => {
    setEmail('');
    setPassword('');
    setFullName('');
    setError(null);
    setPasswordErrors([]);
    clearError();
    onClose();
  };

  // Handle signup submit
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validate password before submitting
    const errors = validatePassword(password);
    if (errors.length > 0) {
      setError('Please fix password requirements');
      return;
    }

    setIsLoading(true);

    try {
      await signup(email, password, fullName || undefined);
      // Success - close modal
      handleClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Signup failed');
    } finally {
      setIsLoading(false);
    }
  };

  // Handle switch to login
  const handleSwitchToLogin = () => {
    handleClose();
    onSwitchToLogin();
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <h2>Sign Up</h2>
          <button className={styles.closeButton} onClick={handleClose}>
            ×
          </button>
        </div>

        <form onSubmit={handleSubmit} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label htmlFor="signup-name">Full Name (Optional)</label>
            <input
              id="signup-name"
              type="text"
              value={fullName}
              onChange={(e) => setFullName(e.target.value)}
              placeholder="John Doe"
              disabled={isLoading}
              className={styles.input}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-email">Email *</label>
            <input
              id="signup-email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="your.email@example.com"
              required
              disabled={isLoading}
              className={styles.input}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-password">Password *</label>
            <input
              id="signup-password"
              type="password"
              value={password}
              onChange={handlePasswordChange}
              placeholder="Create a strong password"
              required
              disabled={isLoading}
              className={styles.input}
            />
          </div>

          {/* Password requirements */}
          {password && (
            <div className={styles.passwordRequirements}>
              <p className={styles.requirementsTitle}>Password must have:</p>
              <ul className={styles.requirementsList}>
                <li className={password.length >= 8 ? styles.met : styles.unmet}>
                  {password.length >= 8 ? '✓' : '○'} At least 8 characters
                </li>
                <li className={/[A-Z]/.test(password) ? styles.met : styles.unmet}>
                  {/[A-Z]/.test(password) ? '✓' : '○'} One uppercase letter
                </li>
                <li className={/[a-z]/.test(password) ? styles.met : styles.unmet}>
                  {/[a-z]/.test(password) ? '✓' : '○'} One lowercase letter
                </li>
                <li className={/\d/.test(password) ? styles.met : styles.unmet}>
                  {/\d/.test(password) ? '✓' : '○'} One digit
                </li>
              </ul>
            </div>
          )}

          {error && (
            <div className={styles.errorMessage}>
              {error}
            </div>
          )}

          <button
            type="submit"
            disabled={isLoading || passwordErrors.length > 0}
            className={styles.submitButton}
          >
            {isLoading ? 'Creating account...' : 'Sign Up'}
          </button>
        </form>

        <div className={styles.switchAuth}>
          Already have an account?{' '}
          <button
            onClick={handleSwitchToLogin}
            className={styles.switchButton}
            type="button"
          >
            Login
          </button>
        </div>
      </div>
    </div>
  );
};

export default SignupModal;
