import React from 'react';
import type { ReactElement } from 'react';
import styles from './styles.module.css';

export type ScopeType = 'page' | 'selection' | 'module' | 'all';

interface ScopeSelectorProps {
  currentScope: ScopeType;
  hasSelection: boolean;
  selectedText?: string;
  onScopeChange: (scope: ScopeType) => void;
  chapterId?: string;
  moduleId?: string;
}

export default function ScopeSelector({
  currentScope,
  hasSelection,
  selectedText,
  onScopeChange,
  chapterId,
  moduleId,
}: ScopeSelectorProps): ReactElement {
  return (
    <div className={styles.scopeSelector}>
      <label htmlFor="scope-select" className={styles.scopeLabel}>
        Search scope:
      </label>
      <select
        id="scope-select"
        className={styles.scopeSelect}
        value={currentScope}
        onChange={(e) => onScopeChange(e.target.value as ScopeType)}
        aria-label="Select search scope"
      >
        {chapterId && (
          <option value="page">Current Page</option>
        )}
        {hasSelection && (
          <option value="selection">Selected Text</option>
        )}
        {moduleId && (
          <option value="module">Current Module</option>
        )}
        <option value="all">All Content</option>
      </select>

      {currentScope === 'selection' && hasSelection && selectedText && (
        <div className={styles.selectionBadge} title={selectedText}>
          <span className={styles.selectionIcon} aria-hidden="true">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="12"
              height="12"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <path d="M17 3a2.828 2.828 0 1 1 4 4L7.5 20.5 2 22l1.5-5.5L17 3z"></path>
            </svg>
          </span>
          <span className={styles.selectionText}>
            "{selectedText.slice(0, 30)}{selectedText.length > 30 ? '...' : ''}"
          </span>
        </div>
      )}
    </div>
  );
}
