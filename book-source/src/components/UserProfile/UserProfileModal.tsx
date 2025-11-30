import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { getUserBackground, submitUserBackground, type UserBackground, type UserBackgroundCreate } from '../../services/userApi';
import styles from './UserProfileModal.module.css';

interface UserProfileModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const UserProfileModal: React.FC<UserProfileModalProps> = ({ isOpen, onClose }) => {
  const { user } = useAuth();
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const [background, setBackground] = useState<UserBackground | null>(null);

  const [formData, setFormData] = useState<UserBackgroundCreate>({
    software_experience: 'beginner',
    hardware_experience: 'beginner',
    robotics_experience: 'none',
    programming_languages: '',
    learning_goals: '',
    current_role: 'student',
    industry: '',
  });

  // Load user background on mount
  useEffect(() => {
    if (isOpen) {
      loadBackground();
    }
  }, [isOpen]);

  const loadBackground = async () => {
    setLoading(true);
    setError(null);
    try {
      const data = await getUserBackground();
      setBackground(data);
      setFormData({
        software_experience: data.software_experience,
        hardware_experience: data.hardware_experience,
        robotics_experience: data.robotics_experience,
        programming_languages: data.programming_languages || '',
        learning_goals: data.learning_goals || '',
        current_role: data.current_role,
        industry: data.industry || '',
      });
    } catch (err) {
      // Background not found is OK - user can create one
      if (err instanceof Error && !err.message.includes('404')) {
        setError(err.message);
      }
    } finally {
      setLoading(false);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSaving(true);
    setError(null);
    setSuccess(false);

    try {
      await submitUserBackground(formData);
      setSuccess(true);
      await loadBackground(); // Reload to get updated data
      setTimeout(() => {
        setSuccess(false);
      }, 3000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save background');
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <h2>Your Learning Profile</h2>
          <button className={styles.closeButton} onClick={onClose}>×</button>
        </div>

        <p className={styles.description}>
          Help us personalize content for you by sharing your experience and goals.
        </p>

        {loading ? (
          <div className={styles.loading}>Loading...</div>
        ) : (
          <form onSubmit={handleSubmit} className={styles.form}>
            <div className={styles.formGroup}>
              <label htmlFor="software_experience">Software Experience</label>
              <select
                id="software_experience"
                name="software_experience"
                value={formData.software_experience}
                onChange={handleChange}
                required
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardware_experience">Hardware Experience</label>
              <select
                id="hardware_experience"
                name="hardware_experience"
                value={formData.hardware_experience}
                onChange={handleChange}
                required
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="robotics_experience">Robotics Experience</label>
              <select
                id="robotics_experience"
                name="robotics_experience"
                value={formData.robotics_experience}
                onChange={handleChange}
                required
              >
                <option value="none">None</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="programming_languages">Programming Languages (comma-separated)</label>
              <input
                type="text"
                id="programming_languages"
                name="programming_languages"
                value={formData.programming_languages}
                onChange={handleChange}
                placeholder="e.g., Python, C++, JavaScript"
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="current_role">Current Role</label>
              <select
                id="current_role"
                name="current_role"
                value={formData.current_role}
                onChange={handleChange}
                required
              >
                <option value="student">Student</option>
                <option value="professional">Professional</option>
                <option value="hobbyist">Hobbyist</option>
                <option value="researcher">Researcher</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="industry">Industry (optional)</label>
              <input
                type="text"
                id="industry"
                name="industry"
                value={formData.industry}
                onChange={handleChange}
                placeholder="e.g., Education, Manufacturing, Research"
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="learning_goals">Learning Goals</label>
              <textarea
                id="learning_goals"
                name="learning_goals"
                value={formData.learning_goals}
                onChange={handleChange}
                rows={4}
                placeholder="What do you want to learn? What are your goals?"
              />
            </div>

            {error && <div className={styles.error}>{error}</div>}
            {success && <div className={styles.success}>Profile saved successfully!</div>}

            <button type="submit" className={styles.submitButton} disabled={saving}>
              {saving ? 'Saving...' : 'Save Profile'}
            </button>
          </form>
        )}
      </div>
    </div>
  );
};

export default UserProfileModal;

