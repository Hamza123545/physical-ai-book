/**
 * Signup Page
 * 
 * Full page registration form using FastAPI JWT auth
 * With toast notifications for errors
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

// Get API base URL
const getApiUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:8000';
  if (window.location.hostname === 'localhost') return 'http://localhost:8000';
  return 'https://physical-ai-backend-9lxv.onrender.com';
};

// Toast notification component
const Toast: React.FC<{ message: string; type: 'error' | 'success'; onClose: () => void }> = ({ message, type, onClose }) => {
  useEffect(() => {
    const timer = setTimeout(onClose, 5000);
    return () => clearTimeout(timer);
  }, [onClose]);

  return (
    <div style={{
      position: 'fixed',
      top: '20px',
      right: '20px',
      padding: '1rem 1.5rem',
      borderRadius: '12px',
      background: type === 'error' ? '#fee2e2' : '#d1fae5',
      color: type === 'error' ? '#dc2626' : '#059669',
      boxShadow: '0 10px 25px rgba(0,0,0,0.15)',
      zIndex: 9999,
      display: 'flex',
      alignItems: 'center',
      gap: '0.75rem',
      maxWidth: '400px',
      animation: 'slideIn 0.3s ease',
    }}>
      <span style={{ fontSize: '1.25rem' }}>
        {type === 'error' ? '⚠️' : '✓'}
      </span>
      <span style={{ flex: 1, fontSize: '0.9rem', fontWeight: 500 }}>{message}</span>
      <button 
        onClick={onClose}
        style={{
          background: 'none',
          border: 'none',
          cursor: 'pointer',
          fontSize: '1.25rem',
          color: 'inherit',
          opacity: 0.7,
        }}
      >
        ×
      </button>
    </div>
  );
};

export default function SignupPage(): React.ReactElement {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [toast, setToast] = useState<{ message: string; type: 'error' | 'success' } | null>(null);
  
  // Personalization fields
  const [showPersonalization, setShowPersonalization] = useState(false);
  const [softwareExperience, setSoftwareExperience] = useState('beginner');
  const [hardwareExperience, setHardwareExperience] = useState('beginner');
  const [roboticsExperience, setRoboticsExperience] = useState('none');
  const [currentRole, setCurrentRole] = useState('student');
  const [programmingLanguages, setProgrammingLanguages] = useState('');
  const [learningGoals, setLearningGoals] = useState('');
  const [industry, setIndustry] = useState('');

  const showError = (message: string) => {
    setToast({ message, type: 'error' });
  };

  const showSuccess = (message: string) => {
    setToast({ message, type: 'success' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validation
    if (password !== confirmPassword) {
      showError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      showError('Password must be at least 8 characters');
      return;
    }

    if (password.length > 64) {
      showError('Password must be less than 64 characters');
      return;
    }

    if (!email.includes('@')) {
      showError('Please enter a valid email address');
      return;
    }

    setIsLoading(true);

    try {
      const signupData: any = { 
        email, 
        password, 
        full_name: name || undefined 
      };
      
      // Add personalization data if provided
      if (showPersonalization && (softwareExperience || hardwareExperience || roboticsExperience || currentRole)) {
        signupData.software_experience = softwareExperience;
        signupData.hardware_experience = hardwareExperience;
        signupData.robotics_experience = roboticsExperience;
        signupData.current_role = currentRole;
        if (programmingLanguages) signupData.programming_languages = programmingLanguages;
        if (learningGoals) signupData.learning_goals = learningGoals;
        if (industry) signupData.industry = industry;
      }
      
      const response = await fetch(`${getApiUrl()}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(signupData),
      });

      const data = await response.json();

      if (!response.ok) {
        // Parse error message from FastAPI response
        let errorMsg = 'Signup failed';
        
        if (data.detail) {
          if (Array.isArray(data.detail)) {
            // Validation errors
            const firstError = data.detail[0];
            if (firstError.msg) {
              errorMsg = firstError.msg;
              if (firstError.loc && firstError.loc[1]) {
                errorMsg = `${firstError.loc[1]}: ${firstError.msg}`;
              }
            }
          } else if (typeof data.detail === 'object' && data.detail.error) {
            errorMsg = data.detail.error.message || 'Signup failed';
          } else if (typeof data.detail === 'string') {
            errorMsg = data.detail;
          }
        }
        
        showError(errorMsg);
        return;
      }

      // Store token and user
      if (data.tokens?.access_token) {
        localStorage.setItem('authToken', data.tokens.access_token);
      }
      if (data.user) {
        localStorage.setItem('user', JSON.stringify(data.user));
      }

      showSuccess('Account created successfully! Redirecting...');
      
      setTimeout(() => {
        window.location.href = '/physical-ai-book/';
      }, 1500);
      
    } catch (err) {
      showError('Network error. Please check your connection.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create an account">
      <style>{`
        @keyframes slideIn {
          from { transform: translateX(100%); opacity: 0; }
          to { transform: translateX(0); opacity: 1; }
        }
        input:focus {
          border-color: var(--ifm-color-primary) !important;
          box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.1) !important;
          outline: none;
        }
      `}</style>
      
      {toast && (
        <Toast 
          message={toast.message} 
          type={toast.type} 
          onClose={() => setToast(null)} 
        />
      )}

      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: 'calc(100vh - 60px)',
        padding: '2rem',
        background: 'linear-gradient(135deg, var(--ifm-background-color) 0%, var(--ifm-color-emphasis-100) 100%)',
      }}>
        <div style={{
          background: 'var(--ifm-background-surface-color)',
          borderRadius: '16px',
          boxShadow: '0 10px 40px rgba(0, 0, 0, 0.1)',
          padding: '3rem',
          width: '100%',
          maxWidth: '420px',
        }}>
          <div style={{ textAlign: 'center', marginBottom: '2rem' }}>
            <h1 style={{ 
              fontSize: '2rem', 
              fontWeight: 700,
              marginBottom: '0.5rem',
              color: 'var(--ifm-color-primary)',
            }}>
              Create Account
            </h1>
            <p style={{ color: 'var(--ifm-color-content-secondary)', margin: 0 }}>
              Start your learning journey
            </p>
          </div>

          <form onSubmit={handleSubmit}>
            <div style={{ marginBottom: '1.25rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 500, fontSize: '0.9rem' }}>
                Name <span style={{ color: 'gray', fontWeight: 400 }}>(optional)</span>
              </label>
              <input
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                disabled={isLoading}
                placeholder="Your name"
                style={{
                  width: '100%',
                  padding: '0.875rem',
                  borderRadius: '10px',
                  border: '2px solid var(--ifm-color-emphasis-300)',
                  fontSize: '1rem',
                  background: 'var(--ifm-background-color)',
                  color: 'var(--ifm-color-content)',
                  transition: 'all 0.2s',
                }}
              />
            </div>

            <div style={{ marginBottom: '1.25rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 500, fontSize: '0.9rem' }}>
                Email
              </label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                disabled={isLoading}
                placeholder="you@example.com"
                style={{
                  width: '100%',
                  padding: '0.875rem',
                  borderRadius: '10px',
                  border: '2px solid var(--ifm-color-emphasis-300)',
                  fontSize: '1rem',
                  background: 'var(--ifm-background-color)',
                  color: 'var(--ifm-color-content)',
                  transition: 'all 0.2s',
                }}
              />
            </div>

            <div style={{ marginBottom: '1.25rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 500, fontSize: '0.9rem' }}>
                Password
              </label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                disabled={isLoading}
                placeholder="Min 8 characters"
                style={{
                  width: '100%',
                  padding: '0.875rem',
                  borderRadius: '10px',
                  border: '2px solid var(--ifm-color-emphasis-300)',
                  fontSize: '1rem',
                  background: 'var(--ifm-background-color)',
                  color: 'var(--ifm-color-content)',
                  transition: 'all 0.2s',
                }}
              />
              <p style={{ margin: '0.5rem 0 0', fontSize: '0.8rem', color: 'var(--ifm-color-content-secondary)' }}>
                8-64 characters
              </p>
            </div>

            <div style={{ marginBottom: '2rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 500, fontSize: '0.9rem' }}>
                Confirm Password
              </label>
              <input
                type="password"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                required
                disabled={isLoading}
                placeholder="Confirm password"
                style={{
                  width: '100%',
                  padding: '0.875rem',
                  borderRadius: '10px',
                  border: '2px solid var(--ifm-color-emphasis-300)',
                  fontSize: '1rem',
                  background: 'var(--ifm-background-color)',
                  color: 'var(--ifm-color-content)',
                  transition: 'all 0.2s',
                }}
              />
            </div>

            {/* Personalization Section */}
            <div style={{ marginBottom: '2rem', padding: '1.5rem', background: 'var(--ifm-color-emphasis-100)', borderRadius: '12px', border: '1px solid var(--ifm-color-emphasis-200)' }}>
              <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: '1rem' }}>
                <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontWeight: 500 }}>
                  <input
                    type="checkbox"
                    checked={showPersonalization}
                    onChange={(e) => setShowPersonalization(e.target.checked)}
                    style={{ width: '18px', height: '18px', cursor: 'pointer' }}
                  />
                  <span>Personalize my learning experience (optional)</span>
                </label>
              </div>
              
              {showPersonalization && (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
                  <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))', gap: '1rem' }}>
                    <div>
                      <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                        Software Experience
                      </label>
                      <select
                        value={softwareExperience}
                        onChange={(e) => setSoftwareExperience(e.target.value)}
                        disabled={isLoading}
                        style={{
                          width: '100%',
                          padding: '0.75rem',
                          borderRadius: '8px',
                          border: '2px solid var(--ifm-color-emphasis-300)',
                          fontSize: '0.9rem',
                          background: 'var(--ifm-background-color)',
                          color: 'var(--ifm-color-content)',
                        }}
                      >
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>
                    
                    <div>
                      <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                        Hardware Experience
                      </label>
                      <select
                        value={hardwareExperience}
                        onChange={(e) => setHardwareExperience(e.target.value)}
                        disabled={isLoading}
                        style={{
                          width: '100%',
                          padding: '0.75rem',
                          borderRadius: '8px',
                          border: '2px solid var(--ifm-color-emphasis-300)',
                          fontSize: '0.9rem',
                          background: 'var(--ifm-background-color)',
                          color: 'var(--ifm-color-content)',
                        }}
                      >
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>
                    
                    <div>
                      <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                        Robotics Experience
                      </label>
                      <select
                        value={roboticsExperience}
                        onChange={(e) => setRoboticsExperience(e.target.value)}
                        disabled={isLoading}
                        style={{
                          width: '100%',
                          padding: '0.75rem',
                          borderRadius: '8px',
                          border: '2px solid var(--ifm-color-emphasis-300)',
                          fontSize: '0.9rem',
                          background: 'var(--ifm-background-color)',
                          color: 'var(--ifm-color-content)',
                        }}
                      >
                        <option value="none">None</option>
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>
                    
                    <div>
                      <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                        Current Role
                      </label>
                      <select
                        value={currentRole}
                        onChange={(e) => setCurrentRole(e.target.value)}
                        disabled={isLoading}
                        style={{
                          width: '100%',
                          padding: '0.75rem',
                          borderRadius: '8px',
                          border: '2px solid var(--ifm-color-emphasis-300)',
                          fontSize: '0.9rem',
                          background: 'var(--ifm-background-color)',
                          color: 'var(--ifm-color-content)',
                        }}
                      >
                        <option value="student">Student</option>
                        <option value="professional">Professional</option>
                        <option value="hobbyist">Hobbyist</option>
                        <option value="researcher">Researcher</option>
                      </select>
                    </div>
                  </div>
                  
                  <div>
                    <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                      Programming Languages (comma-separated)
                    </label>
                    <input
                      type="text"
                      value={programmingLanguages}
                      onChange={(e) => setProgrammingLanguages(e.target.value)}
                      disabled={isLoading}
                      placeholder="e.g., Python, C++, JavaScript"
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        borderRadius: '8px',
                        border: '2px solid var(--ifm-color-emphasis-300)',
                        fontSize: '0.9rem',
                        background: 'var(--ifm-background-color)',
                        color: 'var(--ifm-color-content)',
                      }}
                    />
                  </div>
                  
                  <div>
                    <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                      Learning Goals
                    </label>
                    <textarea
                      value={learningGoals}
                      onChange={(e) => setLearningGoals(e.target.value)}
                      disabled={isLoading}
                      placeholder="What do you want to learn?"
                      rows={3}
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        borderRadius: '8px',
                        border: '2px solid var(--ifm-color-emphasis-300)',
                        fontSize: '0.9rem',
                        background: 'var(--ifm-background-color)',
                        color: 'var(--ifm-color-content)',
                        resize: 'vertical',
                        fontFamily: 'inherit',
                      }}
                    />
                  </div>
                  
                  <div>
                    <label style={{ display: 'block', marginBottom: '0.5rem', fontSize: '0.875rem', fontWeight: 500 }}>
                      Industry (optional)
                    </label>
                    <input
                      type="text"
                      value={industry}
                      onChange={(e) => setIndustry(e.target.value)}
                      disabled={isLoading}
                      placeholder="e.g., Education, Manufacturing, Research"
                      style={{
                        width: '100%',
                        padding: '0.75rem',
                        borderRadius: '8px',
                        border: '2px solid var(--ifm-color-emphasis-300)',
                        fontSize: '0.9rem',
                        background: 'var(--ifm-background-color)',
                        color: 'var(--ifm-color-content)',
                      }}
                    />
                  </div>
                  
                  <p style={{ margin: 0, fontSize: '0.8rem', color: 'var(--ifm-color-content-secondary)', fontStyle: 'italic' }}>
                    You can update these settings later in your profile.
                  </p>
                </div>
              )}
            </div>

            <button
              type="submit"
              disabled={isLoading}
              style={{
                width: '100%',
                padding: '1rem',
                borderRadius: '10px',
                border: 'none',
                background: isLoading ? 'var(--ifm-color-emphasis-400)' : 'var(--ifm-color-primary)',
                color: 'white',
                fontSize: '1rem',
                fontWeight: 600,
                cursor: isLoading ? 'not-allowed' : 'pointer',
                transition: 'all 0.2s',
              }}
            >
              {isLoading ? (
                <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
                  <span style={{ 
                    width: '16px', 
                    height: '16px', 
                    border: '2px solid white', 
                    borderTopColor: 'transparent', 
                    borderRadius: '50%',
                    animation: 'spin 1s linear infinite',
                  }} />
                  Creating account...
                </span>
              ) : 'Create Account'}
            </button>
          </form>

          <div style={{ marginTop: '2rem', textAlign: 'center', borderTop: '1px solid var(--ifm-color-emphasis-200)', paddingTop: '1.5rem' }}>
            <span style={{ color: 'var(--ifm-color-content-secondary)' }}>
              Already have an account?{' '}
            </span>
            <Link to="/login" style={{ color: 'var(--ifm-color-primary)', fontWeight: 600, textDecoration: 'none' }}>
              Sign in
            </Link>
          </div>
        </div>
      </div>

      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
      `}</style>
    </Layout>
  );
}
