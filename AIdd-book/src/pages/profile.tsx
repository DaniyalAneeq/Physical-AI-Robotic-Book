/**
 * User Profile Page
 *
 * Displays user information, onboarding details, and account settings.
 */

import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../hooks/useAuth';
import { getOnboardingProfile, type OnboardingProfile } from '../services/authApi';
import styles from './profile.module.css';

export default function ProfilePage() {
  const { user, loading: authLoading, logout } = useAuth();
  const history = useHistory();
  const [onboardingProfile, setOnboardingProfile] = useState<OnboardingProfile | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Wait for auth to load before redirecting
    if (authLoading) {
      return;
    }

    // Redirect to login if not authenticated
    if (!user) {
      history.push('/Physical-AI-Robotic-Book/login');
      return;
    }

    // Load onboarding profile
    const loadProfile = async () => {
      try {
        const profile = await getOnboardingProfile();
        setOnboardingProfile(profile);
      } catch (error) {
        console.error('Failed to load onboarding profile:', error);
      } finally {
        setLoading(false);
      }
    };

    loadProfile();
  }, [user, authLoading, history]);

  const handleLogout = async () => {
    try {
      await logout();
      history.push('/Physical-AI-Robotic-Book/');
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  // Show loading state while auth is loading
  if (authLoading) {
    return (
      <Layout title="Profile" description="User Profile">
        <div className={styles.container}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  // Don't render anything if not authenticated (redirect will happen in useEffect)
  if (!user) {
    return null;
  }

  return (
    <Layout title="Profile" description="User Profile">
      <div className={styles.container}>
        <div className={styles.profileCard}>
          {/* Header Section */}
          <div className={styles.header}>
            <div className={styles.avatarSection}>
              <div className={styles.avatar}>
                {user.name.charAt(0).toUpperCase()}
              </div>
              <div className={styles.userInfo}>
                <h1 className={styles.userName}>{user.name}</h1>
                <p className={styles.userEmail}>{user.email}</p>
                {user.email_verified_at && (
                  <span className={styles.verifiedBadge}>✓ Verified</span>
                )}
              </div>
            </div>
            <button onClick={handleLogout} className={styles.logoutButton}>
              Sign Out
            </button>
          </div>

          {/* Account Information */}
          <div className={styles.section}>
            <h2 className={styles.sectionTitle}>Account Information</h2>
            <div className={styles.infoGrid}>
              <div className={styles.infoItem}>
                <span className={styles.label}>Member Since</span>
                <span className={styles.value}>
                  {new Date(user.created_at).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric',
                  })}
                </span>
              </div>
              <div className={styles.infoItem}>
                <span className={styles.label}>Last Updated</span>
                <span className={styles.value}>
                  {new Date(user.updated_at).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric',
                  })}
                </span>
              </div>
              <div className={styles.infoItem}>
                <span className={styles.label}>Onboarding Status</span>
                <span className={styles.value}>
                  {user.onboarding_completed ? (
                    <span className={styles.completedBadge}>✓ Completed</span>
                  ) : (
                    <span className={styles.pendingBadge}>⏳ Pending</span>
                  )}
                </span>
              </div>
            </div>
          </div>

          {/* Learning Profile */}
          {loading ? (
            <div className={styles.section}>
              <h2 className={styles.sectionTitle}>Learning Profile</h2>
              <p className={styles.loadingText}>Loading...</p>
            </div>
          ) : onboardingProfile ? (
            <div className={styles.section}>
              <h2 className={styles.sectionTitle}>Learning Profile</h2>
              <div className={styles.profileDetails}>
                <div className={styles.detailCard}>
                  <div className={styles.detailIcon}>👤</div>
                  <div className={styles.detailContent}>
                    <h3>User Type</h3>
                    <p>{onboardingProfile.user_type}</p>
                  </div>
                </div>
                <div className={styles.detailCard}>
                  <div className={styles.detailIcon}>🎯</div>
                  <div className={styles.detailContent}>
                    <h3>Area of Interest</h3>
                    <p>{onboardingProfile.area_of_interest}</p>
                  </div>
                </div>
                <div className={styles.detailCard}>
                  <div className={styles.detailIcon}>📊</div>
                  <div className={styles.detailContent}>
                    <h3>Experience Level</h3>
                    <p>{onboardingProfile.experience_level}</p>
                  </div>
                </div>
                {onboardingProfile.topics_of_interest && onboardingProfile.topics_of_interest.length > 0 && (
                  <div className={styles.detailCardWide}>
                    <div className={styles.detailIcon}>📚</div>
                    <div className={styles.detailContent}>
                      <h3>Topics of Interest</h3>
                      <div className={styles.topicTags}>
                        {onboardingProfile.topics_of_interest.map((topic) => (
                          <span key={topic} className={styles.topicTag}>
                            {topic}
                          </span>
                        ))}
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </div>
          ) : user.onboarding_completed ? (
            <div className={styles.section}>
              <h2 className={styles.sectionTitle}>Learning Profile</h2>
              <p className={styles.noDataText}>Onboarding completed but profile data unavailable.</p>
            </div>
          ) : (
            <div className={styles.section}>
              <h2 className={styles.sectionTitle}>Learning Profile</h2>
              <div className={styles.incompleteOnboarding}>
                <p>Complete your onboarding to personalize your learning experience!</p>
                <button
                  onClick={() => history.push('/Physical-AI-Robotic-Book/?onboarding=required')}
                  className={styles.completeButton}
                >
                  Complete Onboarding
                </button>
              </div>
            </div>
          )}

          {/* Account Actions */}
          <div className={styles.section}>
            <h2 className={styles.sectionTitle}>Account Settings</h2>
            <div className={styles.actions}>
              <button className={styles.actionButton} disabled>
                Change Password
                <span className={styles.comingSoon}>Coming Soon</span>
              </button>
              <button className={styles.actionButton} disabled>
                Update Profile
                <span className={styles.comingSoon}>Coming Soon</span>
              </button>
              <button className={styles.actionButton} disabled>
                Privacy Settings
                <span className={styles.comingSoon}>Coming Soon</span>
              </button>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
