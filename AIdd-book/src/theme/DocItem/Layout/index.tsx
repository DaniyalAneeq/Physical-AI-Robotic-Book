import React from 'react';
import type { ReactElement } from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof LayoutType>;

// Chatbot is now handled globally in Root.tsx
// This wrapper just passes through the layout
export default function LayoutWrapper(props: Props): ReactElement {
  return <Layout {...props} />;
}
