/**
 * User Menu Component
 *
 * Dropdown menu showing user info and logout button.
 * Displays when user is authenticated.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './Auth.module.css';

const UserMenu: React.FC = () => {
  const { user, logout } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Handle logout
  const handleLogout = () => {
    logout();
    setIsOpen(false);
  };

  if (!user) return null;

  // Get user display name (full_name or email)
  const displayName = user.full_name || user.email.split('@')[0];

  // Get user initials for avatar
  const getInitials = () => {
    if (user.full_name) {
      const names = user.full_name.split(' ');
      return names.length > 1
        ? `${names[0][0]}${names[1][0]}`.toUpperCase()
        : names[0][0].toUpperCase();
    }
    return user.email[0].toUpperCase();
  };

  return (
    <div className={styles.userMenu} ref={menuRef}>
      <button
        className={styles.userMenuButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <div className={styles.userAvatar}>
          {getInitials()}
        </div>
        <span className={styles.userName}>{displayName}</span>
        <svg
          className={`${styles.chevron} ${isOpen ? styles.chevronUp : ''}`}
          width="12"
          height="12"
          viewBox="0 0 12 12"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M2 4L6 8L10 4"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      </button>

      {isOpen && (
        <div className={styles.userMenuDropdown}>
          <div className={styles.userMenuHeader}>
            <div className={styles.userInfo}>
              <p className={styles.userFullName}>{user.full_name || 'User'}</p>
              <p className={styles.userEmail}>{user.email}</p>
            </div>
          </div>

          <div className={styles.userMenuDivider} />

          <div className={styles.userMenuActions}>
            <button
              onClick={handleLogout}
              className={styles.logoutButton}
            >
              <svg
                width="16"
                height="16"
                viewBox="0 0 16 16"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
                className={styles.logoutIcon}
              >
                <path
                  d="M6 14H3C2.44772 14 2 13.5523 2 13V3C2 2.44772 2.44772 2 3 2H6"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                />
                <path
                  d="M11 11L14 8M14 8L11 5M14 8H6"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
              Logout
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default UserMenu;
