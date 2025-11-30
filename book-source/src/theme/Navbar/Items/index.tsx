/**
 * Custom Navbar Items Component
 * 
 * Extends Docusaurus navbar items to include authentication buttons on the right side
 */

import React, { useEffect, useRef } from 'react';
import NavbarItems from '@theme-original/Navbar/Items';
import AuthButtons from '@/components/Auth/AuthButtons';
import { AuthProvider } from '@/contexts/AuthContext';
import { createRoot, Root } from 'react-dom/client';
import styles from './styles.module.css';

export default function NavbarItemsWrapper(props: any) {
  const rootRef = useRef<Root | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const injectedRef = useRef(false);

  useEffect(() => {
    // Find the right-side navbar items container with multiple attempts
    const findAndInject = () => {
      // Try multiple selectors
      const selectors = [
        '.navbar__items--right',
        '.navbar__items.navbar__items--right',
        '[class*="navbar__items"][class*="right"]'
      ];

      let rightItems: Element | null = null;
      for (const selector of selectors) {
        rightItems = document.querySelector(selector);
        if (rightItems) break;
      }

      if (rightItems && !injectedRef.current) {
        console.log('Found right navbar items, injecting auth buttons...');
        
        // Create container for auth buttons
        const authContainer = document.createElement('div');
        authContainer.setAttribute('data-auth-buttons', 'true');
        authContainer.className = styles.authButtonsWrapper;
        authContainer.style.display = 'flex';
        authContainer.style.alignItems = 'center';
        authContainer.style.gap = '0.5rem';
        authContainer.style.marginLeft = '0.5rem';
        
        // Append to right items
        rightItems.appendChild(authContainer);
        console.log('Auth buttons container added to navbar');
        
        // Render AuthButtons wrapped in AuthProvider into the container
        try {
          const root = createRoot(authContainer);
          root.render(
            <AuthProvider>
              <AuthButtons />
            </AuthProvider>
          );
          rootRef.current = root;
          containerRef.current = authContainer;
          injectedRef.current = true;
          console.log('Auth buttons rendered successfully');
        } catch (error) {
          console.error('Error rendering auth buttons:', error);
        }
      } else if (!rightItems) {
        console.log('Right navbar items not found yet, retrying...');
      }
    };

    // Try multiple times with increasing delays
    findAndInject();
    const timers = [
      setTimeout(findAndInject, 100),
      setTimeout(findAndInject, 500),
      setTimeout(findAndInject, 1000),
      setTimeout(findAndInject, 2000)
    ];

    // Also use MutationObserver to watch for navbar changes
    const observer = new MutationObserver(() => {
      if (!injectedRef.current) {
        findAndInject();
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });

    return () => {
      timers.forEach(timer => clearTimeout(timer));
      observer.disconnect();
      if (rootRef.current && containerRef.current) {
        try {
          rootRef.current.unmount();
          containerRef.current.remove();
        } catch (e) {
          console.error('Error cleaning up auth buttons:', e);
        }
      }
    };
  }, []);

  return <NavbarItems {...props} />;
}

