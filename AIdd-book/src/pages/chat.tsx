import React from 'react';
import Layout from '@theme/Layout';
import TextbookChat from '@site/src/components/TextbookChat';

export default function ChatPage() {
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
              <TextbookChat />
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
