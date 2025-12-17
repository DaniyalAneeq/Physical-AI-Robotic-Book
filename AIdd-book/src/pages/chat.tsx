import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import TextbookChat from '@site/src/components/TextbookChat';
import { useAuth } from '@site/src/hooks/useAuth';

export default function ChatPage() {
  const { user, loading } = useAuth();

  return (
    <Layout
      title="Textbook Assistant"
      description="Ask questions about Physical AI & Humanoid Robotics">
      <main style={{ padding: '2rem 0', minHeight: '80vh' }}>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <h1 style={{ textAlign: 'center', marginBottom: '2rem' }}>
                Ask the Textbook Assistant
              </h1>
              <p style={{ textAlign: 'center', marginBottom: '2rem', color: 'var(--ifm-color-emphasis-700)' }}>
                Get instant answers about ROS 2, Digital Twins, NVIDIA Isaac Sim, and Vision-Language-Action models.
                The assistant searches through all textbook content to provide accurate, grounded responses with citations.
              </p>

              {!loading && !user ? (
                <div style={{
                  backgroundColor: 'var(--ifm-color-warning-contrast-background)',
                  border: '2px solid var(--ifm-color-warning)',
                  borderRadius: '8px',
                  padding: '2rem',
                  textAlign: 'center',
                  marginBottom: '2rem',
                  maxWidth: '600px',
                  margin: '0 auto 2rem'
                }}>
                  <h3 style={{ marginBottom: '1rem', color: 'var(--ifm-font-color-base)' }}>
                    Authentication Required
                  </h3>
                  <p style={{ marginBottom: '1.5rem', color: 'var(--ifm-color-emphasis-700)' }}>
                    Please log in or create an account to use the Textbook Assistant chatbot.
                  </p>
                  <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center', flexWrap: 'wrap' }}>
                    <Link
                      to="/login"
                      className="button button--primary button--lg"
                    >
                      Log In
                    </Link>
                    <Link
                      to="/register"
                      className="button button--secondary button--lg"
                    >
                      Create Account
                    </Link>
                  </div>
                </div>
              ) : (
                <TextbookChat />
              )}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
