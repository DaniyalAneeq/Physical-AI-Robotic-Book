---
id: 0002
title: Automated QA Workflow Planning
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-automated-qa-workflow
branch: 001-automated-qa-workflow
user: developer
command: /sp.plan
labels: ["architecture", "qa-automation", "testing", "playwright", "httpx"]
links:
  spec: specs/001-automated-qa-workflow/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-automated-qa-workflow/plan.md
  - specs/001-automated-qa-workflow/research.md
  - specs/001-automated-qa-workflow/data-model.md
  - specs/001-automated-qa-workflow/quickstart.md
  - specs/001-automated-qa-workflow/contracts/issue-schema.json
  - specs/001-automated-qa-workflow/contracts/test-scenario-schema.json
  - specs/001-automated-qa-workflow/contracts/workflow-config-schema.json
  - specs/001-automated-qa-workflow/contracts/report-schema.json
  - CLAUDE.md (agent context updated)
tests:
  - Constitution Check: PASS (all gates satisfied)
  - Technology decisions validated against official documentation
---

## Prompt

Create a technical implementation plan for the "Automated Quality Assurance Workflow" based on the approved spec.

The plan must:
- Define the end-to-end workflow: detect → analyze → resolve → re-test → report
- Specify the architecture for:
  - Frontend browser testing (using Playwright or Cypress)
  - Backend API testing (FastAPI endpoints, auth-protected routes)
- Describe how browser tests will:
  - Run in a real headless browser
  - Handle authentication flows (Google OAuth / Better Auth)
  - Capture screenshots, traces, and logs
- Describe how backend tests will:
  - Validate API responses, errors, and performance
- Define how detected issues are:
  - Collected into a structured issues file
  - Categorized (frontend, backend, performance)
- Define the analysis layer:
  - Severity assignment
  - Root-cause grouping
- Define the resolution layer:
  - Auto-fix strategy for safe issues
  - Guidance generation for non-fixable issues
- Define the re-testing and regression loop
- Include failure handling, safety guards, and rollback rules
- Output artifacts (reports, logs, readiness status)

## Response snapshot

Created comprehensive implementation plan for automated QA workflow with complete architecture and design decisions.

**Plan Components Created**:

1. **Technical Architecture** (plan.md - 1,148 lines):
   - End-to-end workflow: DETECT → ANALYZE → RESOLVE → RETEST → REPORT
   - Frontend testing with Playwright (browser automation, auth handling, artifact capture)
   - Backend testing with httpx (API validation, schema checking, performance metrics)
   - Issue collection & categorization with Pydantic models
   - Analysis layer (severity assignment, root cause ID, issue grouping)
   - Resolution layer (auto-fix with safety guards, manual guidance generation)
   - Re-testing & regression detection with loop termination criteria
   - Failure handling for environment, auth, timeout, and fix failures
   - Safety guards: backups, syntax validation, test verification, rollback
   - Output artifacts: JSON reports, HTML dashboard, deployment readiness status, logs
   - CI/CD integration examples (GitHub Actions)
   - Better Auth integration (no custom auth logic)

2. **Technology Research** (research.md - 547 lines):
   - Playwright best practices for SPA testing (Page Object Model, auto-waiting, parallelization)
   - httpx async patterns for FastAPI testing (connection pooling, parallel requests, retry logic)
   - OpenAPI schema validation (jsonschema + openapi-spec-validator)
   - HTML report templating with Jinja2 (template inheritance, custom filters, embedded visualizations)
   - Automated fix patterns (AST transformations, external tools, pattern matching, safety mechanisms)
   - Performance optimization (parallel execution, context reuse, result caching)
   - Error handling strategies (fail-fast vs. continue, retry with backoff, circuit breaker)

3. **Data Model** (data-model.md - 500+ lines):
   - 5 core entities: Issue, TestScenario, TestResult, ResolutionAction, WorkflowExecution
   - All entities with Pydantic models, validation rules, and relationships
   - Entity Relationship Diagram showing all connections
   - Validation examples (valid and invalid cases)
   - Storage & serialization patterns (JSON files + optional PostgreSQL schema)

4. **JSON Schema Contracts**:
   - `contracts/issue-schema.json` (Issue entity schema)
   - `contracts/test-scenario-schema.json` (TestScenario entity schema)
   - `contracts/workflow-config-schema.json` (Configuration schema)
   - `contracts/report-schema.json` (Report output schema)

5. **Quickstart Guide** (quickstart.md - 440+ lines):
   - Prerequisites and installation instructions
   - Configuration setup (environment variables, YAML configs, test scenarios)
   - Running first workflow (basic usage, dry-run mode, custom config)
   - Understanding results (JSON report, HTML dashboard, deployment readiness)
   - Advanced usage (CI/CD integration, programmatic API, custom scenarios)
   - Troubleshooting guide (common issues and solutions)

**Project Structure Defined**:
```
qa-automation/
├── src/
│   ├── detectors/        # Frontend, backend, performance detection
│   ├── analyzers/        # Severity, root cause, grouping
│   ├── resolvers/        # Auto-fix, guidance, validation
│   ├── models/          # Pydantic data models
│   ├── services/        # Workflow engine, auth, artifacts, reporter
│   ├── cli/            # Command-line interface
│   └── lib/            # Public API
├── tests/
│   ├── fixtures/       # Sample apps with known issues
│   ├── unit/
│   ├── integration/
│   └── contract/
├── config/            # YAML configuration files
└── pyproject.toml     # Dependencies
```

**Technology Stack Finalized**:
- Python 3.11+ (matching project standard)
- Playwright 1.40+ (browser automation)
- httpx 0.25+ (async HTTP client)
- FastAPI 0.115+ (test orchestration)
- Pydantic 2.5+ (data validation)
- pytest 7.4+ (testing framework)
- Jinja2 3.1+ (HTML reports)

**Constitution Check**: PASS
- All accuracy, quality, integration, currency, and security gates satisfied
- No violations requiring justification
- Better Auth integration uses existing endpoints (no custom auth logic)
- Read-only testing (no database modifications)

## Outcome

- ✅ Impact: Completed production-ready implementation plan for automated QA workflow with comprehensive architecture, technology decisions, and design patterns
- 🧪 Tests: Constitution Check validated - all gates passed (technical accuracy, code quality, integration coherence, authentication security)
- 📁 Files: Created plan.md (1,148 lines), research.md (547 lines), data-model.md (500+ lines), quickstart.md (440+ lines), 4 JSON schema contracts, updated agent context
- 🔁 Next prompts: Ready for /sp.tasks to generate task breakdown and test cases for implementation
- 🧠 Reflection: Chose Playwright over Cypress for superior multi-browser support and auto-waiting. Selected httpx over requests for async/await compatibility essential for parallel testing. Designed safety-first resolution layer with backups, rollback, and validation at every step. Comprehensive research phase resolved all unknowns upfront, enabling detailed architectural decisions in Phase 1.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all phases (Phase 0 research, Phase 1 design) completed
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates satisfied), Technology validation - PASS (all patterns verified against official docs)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (research) and Phase 1 (design) as specified
- Next experiment (smallest change to try): Proceed to /sp.tasks to break down implementation into testable, executable tasks with acceptance criteria
