/**
 * Custom Navbar Items Component
 * 
 * Simple passthrough - auth buttons are now handled via custom navbar item type
 */

import React from 'react';
import NavbarItems from '@theme-original/Navbar/Items';

export default function NavbarItemsWrapper(props: any) {
  return <NavbarItems {...props} />;
}
