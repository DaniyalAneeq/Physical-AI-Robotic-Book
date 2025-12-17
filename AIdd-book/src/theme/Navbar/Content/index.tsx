/**
 * Custom Navbar Content Component
 *
 * Adds authentication UI to the navbar.
 * Extends the default Docusaurus navbar with Login/Register buttons or UserMenu.
 */

import React from 'react';
import {useThemeConfig, ErrorCauseBoundary} from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem, {type Props as NavbarItemConfig} from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import Link from '@docusaurus/Link';
import UserMenu from '../../../components/Auth/UserMenu';
import { useAuth } from '../../../hooks/useAuth';

import styles from './styles.module.css';

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items as NavbarItemConfig[];
}

function NavbarItems({items}: {items: NavbarItemConfig[]}): JSX.Element {
  return (
    <>
      {items.map((item, i) => (
        <ErrorCauseBoundary
          key={i}
          onError={(error) =>
            new Error(
              `A theme navbar item failed to render.
Please double-check the following navbar item (themeConfig.navbar.items) of your Docusaurus config:
${JSON.stringify(item, null, 2)}`,
              {cause: error},
            )
          }>
          <NavbarItem {...item} />
        </ErrorCauseBoundary>
      ))}
    </>
  );
}

function NavbarContentLayout({
  left,
  right,
}: {
  left: React.ReactNode;
  right: React.ReactNode;
}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

export default function NavbarContent(): JSX.Element {
  const mobileSidebar = useNavbarMobileSidebar();
  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);

  // Safely access auth context with fallback
  let user = null;
  let loading = true;
  try {
    const auth = useAuth();
    user = auth.user;
    loading = auth.loading;
  } catch (error) {
    // Auth context not available yet, use defaults
    console.warn('Auth context not available in navbar:', error);
  }

  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        <>
          <NavbarItems items={rightItems} />
          <NavbarColorModeToggle className={styles.colorModeToggle} />
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}

          {/* Authentication UI */}
          {!loading && (
            <div className={styles.authSection}>
              {user ? (
                <UserMenu />
              ) : (
                <div className={styles.authButtons}>
                  <Link to="/Physical-AI-Robotic-Book/login" className={styles.loginButton}>
                    Log In
                  </Link>
                  <Link to="/Physical-AI-Robotic-Book/register" className={styles.registerButton}>
                    Sign Up
                  </Link>
                </div>
              )}
            </div>
          )}
        </>
      }
    />
  );
}
