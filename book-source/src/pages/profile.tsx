/**
 * Profile Settings Page
 *
 * Dedicated page for users to manage their learning profile.
 * Used for content personalization.
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../contexts/AuthContext';
import { getUserBackground, submitUserBackground, type UserBackground, type UserBackgroundCreate } from '../services/userApi';

// Inline styles for the profile page
const styles = {
  container: {
    minHeight: 'calc(100vh - 60px)',
    display: 'flex',
    alignItems: 'flex-start',
    justifyContent: 'center',
    padding: '3rem 1rem',
    background: 'linear-gradient(135deg, var(--ifm-background-color) 0%, var(--ifm-color-emphasis-100) 100%)',
  },
  card: {
    width: '100%',
    maxWidth: '700px',
    background: 'var(--ifm-background-color)',
    borderRadius: '16px',
    boxShadow: '0 10px 40px rgba(0, 0, 0, 0.1)',
    overflow: 'hidden',
    border: '1px solid var(--ifm-color-emphasis-200)',
  },
  header: {
    padding: '2rem 2.5rem',
    background: 'linear-gradient(135deg, #6366f1 0%, #8b5cf6 100%)',
    color: 'white',
  },
  headerTitle: {
    margin: '0 0 0.5rem 0',
    fontSize: '1.75rem',
    fontWeight: 700,
    display: 'flex',
    alignItems: 'center',
    gap: '0.75rem',
  },
  headerSubtitle: {
    margin: 0,
    fontSize: '1rem',
    opacity: 0.9,
  },
  userInfo: {
    display: 'flex',
    alignItems: 'center',
    gap: '1rem',
    marginTop: '1.5rem',
    padding: '1rem',
    background: 'rgba(255, 255, 255, 0.1)',
    borderRadius: '12px',
  },
  avatar: {
    width: '56px',
    height: '56px',
    borderRadius: '50%',
    background: 'rgba(255, 255, 255, 0.2)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: '1.5rem',
    fontWeight: 700,
  },
  userDetails: {
    flex: 1,
  },
  userName: {
    margin: 0,
    fontSize: '1.125rem',
    fontWeight: 600,
  },
  userEmail: {
    margin: 0,
    fontSize: '0.875rem',
    opacity: 0.8,
  },
  content: {
    padding: '2rem 2.5rem',
  },
  sectionTitle: {
    fontSize: '1.125rem',
    fontWeight: 600,
    color: 'var(--ifm-heading-color)',
    marginBottom: '1.5rem',
    paddingBottom: '0.75rem',
    borderBottom: '2px solid var(--ifm-color-primary)',
    display: 'flex',
    alignItems: 'center',
    gap: '0.5rem',
  },
  formGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
    gap: '1.5rem',
    marginBottom: '2rem',
  },
  formGroup: {
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '0.5rem',
  },
  label: {
    fontSize: '0.875rem',
    fontWeight: 600,
    color: 'var(--ifm-color-content)',
  },
  labelHint: {
    fontSize: '0.75rem',
    fontWeight: 400,
    color: 'var(--ifm-color-content-secondary)',
    marginLeft: '0.25rem',
  },
  select: {
    padding: '0.875rem 1rem',
    fontSize: '1rem',
    border: '2px solid var(--ifm-color-emphasis-300)',
    borderRadius: '10px',
    background: 'var(--ifm-background-color)',
    color: 'var(--ifm-color-content)',
    cursor: 'pointer',
    transition: 'all 0.2s',
  },
  input: {
    padding: '0.875rem 1rem',
    fontSize: '1rem',
    border: '2px solid var(--ifm-color-emphasis-300)',
    borderRadius: '10px',
    background: 'var(--ifm-background-color)',
    color: 'var(--ifm-color-content)',
    transition: 'all 0.2s',
  },
  textarea: {
    padding: '0.875rem 1rem',
    fontSize: '1rem',
    border: '2px solid var(--ifm-color-emphasis-300)',
    borderRadius: '10px',
    background: 'var(--ifm-background-color)',
    color: 'var(--ifm-color-content)',
    resize: 'vertical' as const,
    minHeight: '120px',
    fontFamily: 'inherit',
    transition: 'all 0.2s',
  },
  fullWidth: {
    gridColumn: '1 / -1',
  },
  buttonGroup: {
    display: 'flex',
    gap: '1rem',
    justifyContent: 'flex-end',
    marginTop: '1rem',
    paddingTop: '1.5rem',
    borderTop: '1px solid var(--ifm-color-emphasis-200)',
  },
  submitButton: {
    padding: '0.875rem 2rem',
    fontSize: '1rem',
    fontWeight: 600,
    background: 'linear-gradient(135deg, #6366f1 0%, #8b5cf6 100%)',
    color: 'white',
    border: 'none',
    borderRadius: '10px',
    cursor: 'pointer',
    transition: 'all 0.2s',
    display: 'flex',
    alignItems: 'center',
    gap: '0.5rem',
  },
  backButton: {
    padding: '0.875rem 1.5rem',
    fontSize: '1rem',
    fontWeight: 500,
    background: 'transparent',
    color: 'var(--ifm-color-content-secondary)',
    border: '2px solid var(--ifm-color-emphasis-300)',
    borderRadius: '10px',
    cursor: 'pointer',
    transition: 'all 0.2s',
  },
  alert: {
    padding: '1rem 1.25rem',
    borderRadius: '10px',
    marginBottom: '1.5rem',
    display: 'flex',
    alignItems: 'center',
    gap: '0.75rem',
    fontSize: '0.9375rem',
  },
  errorAlert: {
    background: 'rgba(239, 68, 68, 0.1)',
    color: '#dc2626',
    border: '1px solid rgba(239, 68, 68, 0.2)',
  },
  successAlert: {
    background: 'rgba(34, 197, 94, 0.1)',
    color: '#16a34a',
    border: '1px solid rgba(34, 197, 94, 0.2)',
  },
  infoAlert: {
    background: 'rgba(59, 130, 246, 0.1)',
    color: '#2563eb',
    border: '1px solid rgba(59, 130, 246, 0.2)',
  },
  loading: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    padding: '3rem',
    color: 'var(--ifm-color-content-secondary)',
    gap: '0.75rem',
  },
  spinner: {
    width: '24px',
    height: '24px',
    border: '3px solid var(--ifm-color-emphasis-300)',
    borderTopColor: 'var(--ifm-color-primary)',
    borderRadius: '50%',
    animation: 'spin 0.8s linear infinite',
  },
  notLoggedIn: {
    textAlign: 'center' as const,
    padding: '3rem',
  },
  loginButton: {
    display: 'inline-block',
    marginTop: '1rem',
    padding: '0.75rem 1.5rem',
    background: 'var(--ifm-color-primary)',
    color: 'white',
    textDecoration: 'none',
    borderRadius: '8px',
    fontWeight: 500,
  },
};

export default function ProfilePage(): React.ReactElement {
  const history = useHistory();
  const { isAuthenticated, user } = useAuth();
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const [hasBackground, setHasBackground] = useState(false);

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
    if (isAuthenticated) {
      loadBackground();
    } else {
      setLoading(false);
    }
  }, [isAuthenticated]);

  const loadBackground = async () => {
    setLoading(true);
    setError(null);
    try {
      const data = await getUserBackground();
      setHasBackground(true);
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
      setHasBackground(true);
      setTimeout(() => setSuccess(false), 5000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save profile');
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const getInitials = () => {
    if (user?.full_name) {
      const names = user.full_name.split(' ');
      return names.length > 1
        ? `${names[0][0]}${names[1][0]}`.toUpperCase()
        : names[0][0].toUpperCase();
    }
    return user?.email?.[0].toUpperCase() || 'U';
  };

  return (
    <Layout title="Profile Settings" description="Manage your learning profile">
      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
        .profile-select:focus,
        .profile-input:focus,
        .profile-textarea:focus {
          outline: none;
          border-color: var(--ifm-color-primary);
          box-shadow: 0 0 0 4px rgba(99, 102, 241, 0.1);
        }
        .profile-button:hover:not(:disabled) {
          transform: translateY(-2px);
          box-shadow: 0 4px 12px rgba(99, 102, 241, 0.3);
        }
        .profile-button:disabled {
          opacity: 0.6;
          cursor: not-allowed;
        }
        .back-button:hover {
          background: var(--ifm-color-emphasis-100);
          border-color: var(--ifm-color-emphasis-400);
        }
      `}</style>

      <div style={styles.container}>
        <div style={styles.card}>
          {/* Header */}
          <div style={styles.header}>
            <h1 style={styles.headerTitle}>
              <span>⚙️</span> Profile Settings
            </h1>
            <p style={styles.headerSubtitle}>
              Customize your learning experience with personalized content
            </p>

            {isAuthenticated && user && (
              <div style={styles.userInfo}>
                <div style={styles.avatar}>{getInitials()}</div>
                <div style={styles.userDetails}>
                  <p style={styles.userName}>{user.full_name || 'User'}</p>
                  <p style={styles.userEmail}>{user.email}</p>
                </div>
              </div>
            )}
          </div>

          {/* Content */}
          <div style={styles.content}>
            {!isAuthenticated ? (
              <div style={styles.notLoggedIn}>
                <span style={{ fontSize: '3rem' }}>🔒</span>
                <h3>Please Log In</h3>
                <p>You need to be logged in to manage your profile.</p>
                <Link to="/login" style={styles.loginButton}>
                  Go to Login
                </Link>
              </div>
            ) : loading ? (
              <div style={styles.loading}>
                <div style={styles.spinner}></div>
                <span>Loading your profile...</span>
              </div>
            ) : (
              <form onSubmit={handleSubmit}>
                {error && (
                  <div style={{ ...styles.alert, ...styles.errorAlert }}>
                    <span>❌</span> {error}
                  </div>
                )}

                {success && (
                  <div style={{ ...styles.alert, ...styles.successAlert }}>
                    <span>✅</span> Profile saved successfully! Your content will now be personalized.
                  </div>
                )}

                {!hasBackground && !success && (
                  <div style={{ ...styles.alert, ...styles.infoAlert }}>
                    <span>💡</span> Set up your profile to get personalized content tailored to your experience level.
                  </div>
                )}

                {/* Experience Section */}
                <h3 style={styles.sectionTitle}>
                  <span>📊</span> Your Experience
                </h3>

                <div style={styles.formGrid}>
                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Software Experience
                      <span style={styles.labelHint}>(coding, programming)</span>
                    </label>
                    <select
                      name="software_experience"
                      value={formData.software_experience}
                      onChange={handleChange}
                      style={styles.select}
                      className="profile-select"
                      required
                    >
                      <option value="beginner">🌱 Beginner - Just starting out</option>
                      <option value="intermediate">📈 Intermediate - Some experience</option>
                      <option value="advanced">🚀 Advanced - Very experienced</option>
                    </select>
                  </div>

                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Hardware Experience
                      <span style={styles.labelHint}>(electronics, circuits)</span>
                    </label>
                    <select
                      name="hardware_experience"
                      value={formData.hardware_experience}
                      onChange={handleChange}
                      style={styles.select}
                      className="profile-select"
                      required
                    >
                      <option value="beginner">🌱 Beginner - New to hardware</option>
                      <option value="intermediate">📈 Intermediate - Built some projects</option>
                      <option value="advanced">🚀 Advanced - Expert level</option>
                    </select>
                  </div>

                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Robotics Experience
                      <span style={styles.labelHint}>(ROS, simulation, robots)</span>
                    </label>
                    <select
                      name="robotics_experience"
                      value={formData.robotics_experience}
                      onChange={handleChange}
                      style={styles.select}
                      className="profile-select"
                      required
                    >
                      <option value="none">🆕 None - Completely new</option>
                      <option value="beginner">🌱 Beginner - Basic knowledge</option>
                      <option value="intermediate">📈 Intermediate - Some projects</option>
                      <option value="advanced">🚀 Advanced - Professional experience</option>
                    </select>
                  </div>

                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Current Role
                      <span style={styles.labelHint}>(helps us tailor examples)</span>
                    </label>
                    <select
                      name="current_role"
                      value={formData.current_role}
                      onChange={handleChange}
                      style={styles.select}
                      className="profile-select"
                      required
                    >
                      <option value="student">🎓 Student</option>
                      <option value="professional">💼 Professional</option>
                      <option value="hobbyist">🛠️ Hobbyist</option>
                      <option value="researcher">🔬 Researcher</option>
                    </select>
                  </div>
                </div>

                {/* Skills Section */}
                <h3 style={styles.sectionTitle}>
                  <span>🛠️</span> Skills & Goals
                </h3>

                <div style={styles.formGrid}>
                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Programming Languages
                      <span style={styles.labelHint}>(comma-separated)</span>
                    </label>
                    <input
                      type="text"
                      name="programming_languages"
                      value={formData.programming_languages}
                      onChange={handleChange}
                      style={styles.input}
                      className="profile-input"
                      placeholder="e.g., Python, C++, JavaScript, Rust"
                    />
                  </div>

                  <div style={styles.formGroup}>
                    <label style={styles.label}>
                      Industry
                      <span style={styles.labelHint}>(optional)</span>
                    </label>
                    <input
                      type="text"
                      name="industry"
                      value={formData.industry}
                      onChange={handleChange}
                      style={styles.input}
                      className="profile-input"
                      placeholder="e.g., Education, Manufacturing, Healthcare"
                    />
                  </div>

                  <div style={{ ...styles.formGroup, ...styles.fullWidth }}>
                    <label style={styles.label}>
                      Learning Goals
                      <span style={styles.labelHint}>(what do you want to achieve?)</span>
                    </label>
                    <textarea
                      name="learning_goals"
                      value={formData.learning_goals}
                      onChange={handleChange}
                      style={styles.textarea}
                      className="profile-textarea"
                      placeholder="Tell us about your goals...&#10;&#10;Examples:&#10;• Build a robot for my university project&#10;• Learn ROS2 for my job&#10;• Understand AI in robotics for research"
                    />
                  </div>
                </div>

                {/* Buttons */}
                <div style={styles.buttonGroup}>
                  <button
                    type="button"
                    onClick={() => history.goBack()}
                    style={styles.backButton}
                    className="back-button"
                  >
                    ← Back
                  </button>
                  <button
                    type="submit"
                    style={styles.submitButton}
                    className="profile-button"
                    disabled={saving}
                  >
                    {saving ? (
                      <>
                        <div style={{ ...styles.spinner, width: '18px', height: '18px', borderWidth: '2px' }}></div>
                        Saving...
                      </>
                    ) : (
                      <>
                        <span>💾</span> Save Profile
                      </>
                    )}
                  </button>
                </div>
              </form>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
}

