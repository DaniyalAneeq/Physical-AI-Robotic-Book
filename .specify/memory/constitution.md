<!--
SYNC IMPACT REPORT
==================
Version change: 1.0.0 → 1.1.0 (MINOR - new section added)

Modified principles: None (all principles retained)

Added sections:
  - Section VII: Integrated RAG Chatbot Development
  - Section VIII: Content Workflow (RAG artifacts integrated)
  - Section IX: Quality Gates (RAG-specific quality criteria added)

Removed sections: None

Templates requiring updates:
  - .specify/templates/plan-template.md → ✅ No update required (Constitution Check references constitution dynamically)
  - .specify/templates/spec-template.md → ✅ No update required (generic template)
  - .specify/templates/tasks-template.md → ✅ No update required (generic template)

Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy

All content MUST be technically accurate and verifiable. Code examples MUST be executable and tested. API references MUST match official documentation. Outdated or deprecated methods MUST NOT be used without explicit deprecation notices.

**Rationale**: Learners rely on textbook content for production implementations; inaccuracies erode trust and cause downstream failures.

### II. Pedagogical Clarity

Content MUST follow a progressive learning path: fundamentals before advanced topics. Each chapter MUST include learning objectives, key concepts, and practical exercises. Diagrams MUST be described with sufficient detail for visual learners and accessibility.

**Rationale**: Effective learning requires scaffolded instruction that builds on prior knowledge.

### III. Consistency & Standards

All content MUST adhere to consistent formatting, naming conventions, and structural patterns across modules. Docusaurus markdown conventions MUST be followed. File naming MUST be deterministic (e.g., `01-introduction.md`, `02-setup.md`).

**Rationale**: Consistency reduces cognitive load and enables automated tooling.

### IV. Code Quality

All code examples MUST follow language-specific best practices (PEP 8 for Python, ROS 2 conventions for rclpy). Examples MUST include error handling where appropriate. Dependencies MUST be explicitly versioned.

**Rationale**: Learners adopt patterns from examples; poor examples propagate poor practices.

### V. Integration Coherence

All modules MUST build toward the unified capstone project (Autonomous Humanoid). Cross-module references MUST be explicit. Prerequisites MUST be clearly stated at each chapter's beginning.

**Rationale**: The textbook forms a coherent learning journey; isolated modules fail the integrated vision.

### VI. Technology Currency

Primary technologies: ROS 2 (Foxy/Humble), Gazebo, Unity Robotics SDK, NVIDIA Isaac Sim, Docusaurus 3.x. All version references MUST specify minimum versions. Deprecated features MUST include migration guidance.

**Rationale**: Robotics evolves rapidly; stale technology references mislead learners.

## Integrated RAG Chatbot Development

### VII. RAG Chatbot Architecture

The textbook MUST include an embedded Retrieval-Augmented Generation (RAG) chatbot that enables learners to query book content interactively.

**Technical Stack**:
- **Reasoning Engine**: OpenAI Agents/ChatKit SDKs for response generation
- **Backend**: FastAPI for API endpoints
- **Database**: Neon Serverless Postgres for embeddings and session data
- **Vector Search**: Qdrant Cloud (Free Tier) for content retrieval
- **Frontend**: Fully embedded in Docusaurus pages with interactive UI

**Core Requirements**:
1. **Content Fidelity**: Responses MUST be grounded exclusively in textbook content. No hallucinated or external information.
2. **Selective Retrieval**: Users MUST be able to scope queries to specific chapters, modules, or user-selected text passages.
3. **Technical Accuracy**: All chatbot responses MUST maintain the same accuracy and pedagogical clarity standards as static content.
4. **Citation Transparency**: Responses MUST include source references (chapter, section, page/paragraph where applicable).

**Rationale**: Interactive Q&A accelerates learning and addresses individual learner questions without requiring instructor availability.

### VIII. RAG Security & Privacy

1. **Query Security**: All database and vector store queries MUST use parameterized queries to prevent injection attacks.
2. **Session Management**: User sessions MUST be managed securely with encrypted tokens. Session data MUST expire after 24 hours of inactivity.
3. **Privacy Compliance**: Logging MUST capture query patterns for analytics without storing personally identifiable information (PII). IP addresses MUST be anonymized.
4. **Rate Limiting**: API endpoints MUST enforce rate limits to prevent abuse (default: 60 requests/minute per session).

**Rationale**: Educational platforms handle learner data; security and privacy are non-negotiable.

### IX. RAG Versioning & Dependencies

All RAG-related components MUST specify explicit versions:
- OpenAI SDK: Minimum version in `requirements.txt`
- FastAPI: Pinned version with patch range
- Qdrant Client: Pinned version
- Neon Postgres driver: Pinned version
- Frontend SDK/components: Pinned in `package.json`

Configuration and setup instructions MUST be documented in `CLAUDE.md` for AI workflow integration.

**Rationale**: Reproducibility requires explicit versioning; implicit versions cause deployment failures.

## Content Workflow

### X. Artifact Sequence

All features, including RAG chatbot development, MUST follow this artifact sequence:

1. **Specification** (`spec.md`): Define scope, user stories, functional requirements, and success criteria.
2. **Plan** (`plan.md`): Document architecture, integration points, UI/UX design, and technical decisions.
3. **Tasks** (`tasks.md`): Break implementation into testable, dependency-ordered tasks.
4. **Implementation**: Execute tasks following constitution principles. Use TDD where specified.
5. **Review**: Validate accuracy, response quality, security compliance, and accessibility.

**RAG-Specific Artifacts**:
- Backend embedding pipeline specification
- Vector store schema and indexing strategy
- Frontend component integration plan
- End-to-end query flow tests

**Rationale**: Structured workflow ensures traceability from requirements to implementation.

## Quality Gates

### XI. Content Quality Gates

All textbook content MUST pass:
- [ ] Technical review by subject matter expert
- [ ] Code execution validation (all examples run without error)
- [ ] Accessibility check (alt text for images, semantic headings)
- [ ] Cross-reference validation (all links resolve)
- [ ] Build validation (`npm run build` succeeds)

### XII. RAG Chatbot Quality Gates

RAG integration MUST pass:
- [ ] **Accuracy Test**: 95% of test queries MUST return correct answers from selected content
- [ ] **Retrieval Precision**: Retrieved chunks MUST be relevant to query (precision > 0.85)
- [ ] **Latency**: Query-to-response time MUST be < 3 seconds (p95)
- [ ] **Build Integration**: Chatbot components MUST not break Docusaurus build
- [ ] **Deployment Check**: Staging deployment MUST pass smoke tests before production
- [ ] **Accessibility**: Chatbot UI MUST be keyboard-navigable and screen-reader compatible (WCAG 2.1 AA)

**Rationale**: Quality gates prevent regressions and ensure consistent learner experience.

## Governance

### XIII. Amendment Process

1. All constitution amendments MUST be documented with rationale.
2. Amendments MUST follow semantic versioning:
   - MAJOR: Backward-incompatible principle changes
   - MINOR: New sections or material expansions
   - PATCH: Clarifications and wording fixes
3. Amendments MUST update dependent templates where necessary.
4. All PRs MUST verify compliance with this constitution.

### XIV. Compliance Review

1. Constitution supersedes all other practices.
2. Complexity MUST be justified in the Complexity Tracking section of implementation plans.
3. Deviations MUST be documented as ADRs with explicit approval.

### XV. RAG Governance

1. RAG chatbot updates MUST comply with all existing constitution principles.
2. Model/SDK upgrades MUST be tested against quality gates before deployment.
3. Embedding schema changes MUST include migration plans.
4. User feedback on chatbot responses MUST be reviewed monthly for accuracy improvements.

**Version**: 1.1.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
