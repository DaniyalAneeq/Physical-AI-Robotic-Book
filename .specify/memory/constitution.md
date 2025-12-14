<!--
SYNC IMPACT REPORT
==================
Version change: 1.1.0 → 1.2.0 (MINOR - authentication section added)

Modified principles:
  - VIII renamed and expanded to include Authentication Security

Added sections:
  - Section X: Authentication & Identity Management (Better Auth)
  - Authentication Quality Gates

Removed sections: None

Templates requiring updates:
  - None (constitution-only change)
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy

All content MUST be technically accurate and verifiable. Code examples MUST be executable and tested. API references MUST match official documentation. Outdated or deprecated methods MUST NOT be used without explicit deprecation notices.

### II. Pedagogical Clarity

Content MUST follow a progressive learning path: fundamentals before advanced topics. Each chapter MUST include learning objectives, key concepts, and practical exercises.

### III. Consistency & Standards

All content MUST adhere to consistent formatting, naming conventions, and structural patterns. Docusaurus markdown conventions MUST be followed.

### IV. Code Quality

All code examples MUST follow language-specific best practices (PEP 8 for Python). Dependencies MUST be explicitly versioned.

### V. Integration Coherence

All modules MUST build toward the unified capstone project. Cross-module references MUST be explicit.

### VI. Technology Currency

Primary technologies include Docusaurus 3.x, FastAPI, OpenAI Agents SDK, Qdrant, and Neon. All versions MUST be pinned or minimally specified.

---

## Integrated RAG Chatbot Development

### VII. RAG Chatbot Architecture

The textbook MUST include an embedded Retrieval-Augmented Generation (RAG) chatbot.

**Technical Stack**:

* Reasoning: OpenAI Agents SDK (required)
* Backend: FastAPI
* Database: Neon Serverless Postgres
* Vector Store: Qdrant Cloud
* Frontend: Docusaurus-integrated UI

### VIII. RAG Security, Privacy & Sessions

1. Parameterized queries only
2. Encrypted session handling
3. No PII storage
4. Rate limiting (60 req/min/session)

---

## Authentication & Identity Management

### IX. Authentication Architecture (Better Auth)

Authentication MUST be implemented using **Better Auth** as the single source of truth.

**Architecture Model**:

* Backend-first authentication via FastAPI
* Frontend (Docusaurus) consumes auth state via HTTP only
* No authentication secrets or logic in frontend code

### X. Authentication Rules

1. Custom authentication flows that replace or duplicate Better Auth are forbidden
2. Only documented Better Auth extensions (callbacks, adapters) are permitted
3. Backend MUST validate identity on every protected request
4. Sessions, cookies, or tokens MUST be:

   * HttpOnly (where applicable)
   * Secure in production
   * Properly expired and rotated

### XI. Authentication Integration Scope

Authentication review MUST include:

* FastAPI auth configuration and routes
* Login, logout, session, and OAuth endpoints
* Token and cookie handling
* Auth-related database schema
* Environment variables

### XII. Authentication Tooling

* Better Auth documentation MUST be retrieved via Context 7 MCP or better-auth MCP
* Authentication changes MUST be reviewed by the `better-auth-expert` agent

---

## Content Workflow

### XIII. Artifact Sequence

All features MUST follow:

1. Specification
2. Plan
3. Tasks
4. Implementation
5. Review

---

## Quality Gates

### XIV. Authentication Quality Gates

Authentication changes MUST pass:

* [ ] No custom auth logic bypassing Better Auth
* [ ] Secure cookie/token configuration
* [ ] Session expiration verified
* [ ] Frontend contains no secrets
* [ ] Auth flows tested in staging

### XV. RAG Chatbot Quality Gates

RAG integration MUST meet accuracy, latency, build, and accessibility thresholds.

---

## Governance

### XVI. Amendment Process

Amendments MUST follow semantic versioning and be documented.

### XVII. Compliance Review

This constitution supersedes all other practices.

---

**Version**: 1.2.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
