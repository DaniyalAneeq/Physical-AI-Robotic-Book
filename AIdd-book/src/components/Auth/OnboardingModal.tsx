/**
 * Onboarding Modal Component
 *
 * Displays a modal popup for collecting user onboarding information.
 */

import React, { useState, useEffect } from 'react';
import { getOnboardingOptions, completeOnboarding, type OnboardingData, type OnboardingOptions } from '../../services/authApi';
import styles from './styles.module.css';

interface OnboardingModalProps {
  isOpen: boolean;
  onComplete: () => void;
  onClose?: () => void;
}

export default function OnboardingModal({ isOpen, onComplete, onClose }: OnboardingModalProps) {
  const [options, setOptions] = useState<OnboardingOptions | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [userType, setUserType] = useState('');
  const [areaOfInterest, setAreaOfInterest] = useState('');
  const [experienceLevel, setExperienceLevel] = useState('');
  const [topicsOfInterest, setTopicsOfInterest] = useState<string[]>([]);

  useEffect(() => {
    if (isOpen) {
      loadOptions();
    }
  }, [isOpen]);

  const loadOptions = async () => {
    try {
      const data = await getOnboardingOptions();
      setOptions(data);
    } catch (err) {
      setError('Failed to load onboarding options');
      console.error(err);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const data: OnboardingData = {
        user_type: userType,
        area_of_interest: areaOfInterest,
        experience_level: experienceLevel,
        topics_of_interest: topicsOfInterest.length > 0 ? topicsOfInterest : undefined,
      };

      await completeOnboarding(data);
      // Call onComplete immediately after successful API call
      onComplete();
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to complete onboarding';

      // If profile already exists, treat it as success (user has already completed onboarding)
      if (errorMsg.includes('already has an onboarding profile') || errorMsg.includes('already has a profile')) {
        console.log('Onboarding already completed, closing modal');
        onComplete();
      } else {
        setError(errorMsg);
      }
    } finally {
      setLoading(false);
    }
  };

  const handleTopicToggle = (topic: string) => {
    setTopicsOfInterest(prev =>
      prev.includes(topic)
        ? prev.filter(t => t !== topic)
        : [...prev, topic]
    );
  };

  if (!isOpen || !options) {
    return null;
  }

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modalContent}>
        <div className={styles.modalHeader}>
          <h2>Welcome! Let's personalize your experience</h2>
          <p>Tell us a bit about yourself to get the most out of our platform.</p>
        </div>

        {error && <div className={styles.error}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.onboardingForm}>
          <div className={styles.formGroup}>
            <label htmlFor="userType">Which describes you best? *</label>
            <select
              id="userType"
              value={userType}
              onChange={(e) => setUserType(e.target.value)}
              required
              disabled={loading}
            >
              <option value="">Select an option</option>
              {options.user_types.map((type) => (
                <option key={type} value={type}>
                  {type}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="areaOfInterest">What's your area of interest? *</label>
            <select
              id="areaOfInterest"
              value={areaOfInterest}
              onChange={(e) => setAreaOfInterest(e.target.value)}
              required
              disabled={loading}
            >
              <option value="">Select an option</option>
              {options.areas_of_interest.map((area) => (
                <option key={area} value={area}>
                  {area}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="experienceLevel">What's your experience level? *</label>
            <select
              id="experienceLevel"
              value={experienceLevel}
              onChange={(e) => setExperienceLevel(e.target.value)}
              required
              disabled={loading}
            >
              <option value="">Select an option</option>
              {options.experience_levels.map((level) => (
                <option key={level} value={level}>
                  {level}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label>Select topics you're interested in (optional):</label>
            <div className={styles.checkboxGroup}>
              {options.topics_of_interest.map((topic) => (
                <label key={topic} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={topicsOfInterest.includes(topic)}
                    onChange={() => handleTopicToggle(topic)}
                    disabled={loading}
                  />
                  <span>{topic}</span>
                </label>
              ))}
            </div>
          </div>

          <div className={styles.modalActions}>
            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading || !userType || !areaOfInterest || !experienceLevel}
            >
              {loading ? 'Saving...' : 'Complete Setup'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}
