# Implementation Plan: Automated Quality Assurance Workflow

**Branch**: `001-automated-qa-workflow` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-automated-qa-workflow/spec.md`

## Summary

Implement an end-to-end automated quality assurance workflow system that detects, analyzes, resolves, and re-tests issues in both frontend (browser) and backend (API) applications before production deployment. The system will use browser automation (Playwright) for frontend testing, HTTP client testing for backend API validation, structured issue tracking with categorization, intelligent analysis with severity assignment, automated fix application with safety guards, and continuous re-testing with regression detection. The workflow outputs deployment readiness reports with full traceability.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: Playwright (browser automation), FastAPI (test orchestration), httpx (API testing), Pydantic (data validation), PyYAML (configuration), pytest (test framework)
**Storage**: JSON files for issue tracking, YAML for configuration, local filesystem for screenshots/traces/logs
**Testing**: pytest for workflow tests, Playwright for browser tests, httpx for API tests
**Target Platform**: Linux/macOS development environments, CI/CD pipeline execution (GitHub Actions, GitLab CI)
**Project Type**: Single project with CLI interface and library API
**Performance Goals**:
- Frontend detection: <15 minutes for 50 pages
- Backend detection: <10 minutes for 100 endpoints
- Analysis: <2 minutes for 500 issues
- Complete workflow: <30 minutes end-to-end
**Constraints**:
- No external dependencies during test execution (offline-capable)
- Must handle authentication flows (OAuth2, session-based)
- Must preserve test artifacts for debugging
- Must support parallel test execution
- Must provide real-time progress updates
**Scale/Scope**:
- 50-100 frontend pages per run
- 100-200 API endpoints per run
- 500-1000 issues detected per run
- 10-50 automated fixes per run

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Technical Accuracy Gates
- [x] All browser automation patterns verified against Playwright documentation
- [x] All API testing patterns verified against httpx/FastAPI documentation
- [x] Code examples will be tested before documentation

### Code Quality Gates
- [x] Python code follows PEP 8 standards
- [x] All dependencies pinned with minimum versions
- [x] Type hints used throughout (Python 3.11+ syntax)

### Integration Coherence Gates
- [x] QA workflow integrates with existing CI/CD (does not replace it)
- [x] Compatible with current project structure (Docusaurus frontend, FastAPI backend)
- [x] Reuses existing authentication mechanisms (Better Auth)

### Technology Currency Gates
- [x] Playwright (latest stable, 1.40+)
- [x] FastAPI (0.115+, matching project standard)
- [x] Python 3.11+ (matching project standard)

### Authentication Security Gates
- [x] No authentication logic bypassing Better Auth
- [x] Test credentials stored securely (environment variables)
- [x] No secrets in code or configuration files
- [x] Frontend auth tested via backend endpoints only

### RAG & Project-Specific Gates
- [x] Does not interfere with RAG chatbot functionality
- [x] Does not modify database schemas (read-only access for testing)
- [x] Respects rate limits on external services

**Constitution Compliance**: PASS - All gates satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-automated-qa-workflow/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Technology decisions & patterns
├── data-model.md        # Phase 1 output - Entity schemas
├── quickstart.md        # Phase 1 output - Setup & usage guide
├── contracts/           # Phase 1 output - API interfaces
│   ├── issue-schema.json
│   ├── test-scenario-schema.json
│   ├── workflow-config-schema.json
│   └── report-schema.json
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
qa-automation/                    # New directory for QA workflow system
├── src/
│   ├── detectors/               # Issue detection modules
│   │   ├── __init__.py
│   │   ├── base.py             # Base detector interface
│   │   ├── frontend.py         # Playwright browser testing
│   │   ├── backend.py          # API testing with httpx
│   │   └── performance.py      # Performance metrics collection
│   ├── analyzers/              # Issue analysis modules
│   │   ├── __init__.py
│   │   ├── severity.py         # Severity assignment logic
│   │   ├── root_cause.py       # Root cause identification
│   │   └── grouping.py         # Related issue grouping
│   ├── resolvers/              # Issue resolution modules
│   │   ├── __init__.py
│   │   ├── auto_fix.py         # Automated fix application
│   │   ├── guidance.py         # Resolution guidance generation
│   │   └── validator.py        # Fix verification
│   ├── models/                 # Data models (Pydantic)
│   │   ├── __init__.py
│   │   ├── issue.py           # Issue entity
│   │   ├── test_scenario.py   # Test scenario entity
│   │   ├── test_result.py     # Test result entity
│   │   ├── resolution.py      # Resolution action entity
│   │   └── workflow.py        # Workflow execution entity
│   ├── services/              # Core workflow services
│   │   ├── __init__.py
│   │   ├── workflow_engine.py # Main workflow orchestration
│   │   ├── auth_handler.py    # Authentication for tests
│   │   ├── artifact_manager.py # Screenshots, traces, logs
│   │   └── reporter.py        # Report generation
│   ├── cli/                   # Command-line interface
│   │   ├── __init__.py
│   │   └── main.py           # CLI entry point
│   └── lib/                   # Public API
│       ├── __init__.py
│       └── workflow.py       # Programmatic workflow API
├── tests/                     # Test suite
│   ├── fixtures/             # Test fixtures & sample apps
│   │   ├── sample_frontend/  # Sample HTML/JS app with known issues
│   │   ├── sample_backend/   # Sample FastAPI app with known issues
│   │   └── auth_mock/        # Mock authentication setup
│   ├── unit/                 # Unit tests
│   │   ├── test_detectors.py
│   │   ├── test_analyzers.py
│   │   ├── test_resolvers.py
│   │   └── test_models.py
│   ├── integration/          # Integration tests
│   │   ├── test_workflow.py
│   │   ├── test_frontend_detection.py
│   │   ├── test_backend_detection.py
│   │   └── test_resolution.py
│   └── contract/             # Contract tests for schemas
│       └── test_schemas.py
├── config/                   # Configuration files
│   ├── default.yaml         # Default workflow configuration
│   ├── frontend-tests.yaml  # Frontend test scenarios
│   └── backend-tests.yaml   # Backend test scenarios
├── pyproject.toml           # Project dependencies
├── README.md                # Project overview
└── .github/
    └── workflows/
        └── qa-workflow.yml  # CI/CD integration example
```

**Structure Decision**: Single project structure chosen because:
1. QA workflow is a self-contained tool that operates on existing applications
2. All components (detection, analysis, resolution) share common models and services
3. Simpler deployment as a single installable package
4. Easier to maintain and test as a cohesive unit
5. Can be invoked from CI/CD or local development environments uniformly

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations - constitution check passed all gates.

---

## Architecture & Design Decisions

### 1. End-to-End Workflow Architecture

**Workflow Phases**:
```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   DETECT    │────▶│   ANALYZE   │────▶│   RESOLVE   │────▶│   RETEST    │────▶│   REPORT    │
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
      │                    │                    │                    │                    │
      ▼                    ▼                    ▼                    ▼                    ▼
  Frontend            Severity           Auto-Fix             Full                Deployment
  Backend            Assignment          Guidance          Regression            Readiness
  Performance        Root Cause          Validation         Testing               Status
                     Grouping
```

**Phase Details**:

1. **DETECT Phase**:
   - Parallel execution: Frontend (Playwright) + Backend (httpx) + Performance (timing)
   - Output: Raw issues with context (screenshots, request/response data, metrics)
   - Duration: 10-15 minutes for typical application

2. **ANALYZE Phase**:
   - Sequential processing: Severity assignment → Root cause → Grouping
   - Output: Prioritized, categorized issues with relationships
   - Duration: 1-2 minutes for 500 issues

3. **RESOLVE Phase**:
   - Parallel where safe: Auto-fixes on independent files
   - Output: Applied fixes + guidance for manual fixes
   - Duration: 2-5 minutes for 50 auto-fixable issues

4. **RETEST Phase**:
   - Full regression: Re-run all detection tests
   - Output: Confirmation of fixes + new issues detected
   - Duration: 10-15 minutes (same as DETECT)

5. **REPORT Phase**:
   - Aggregation: Compile all phases into final report
   - Output: JSON report, HTML dashboard, deployment readiness status
   - Duration: <1 minute

### 2. Frontend Browser Testing (Playwright)

**Architecture**:
```python
# Browser test execution flow
1. Load test scenarios from config (frontend-tests.yaml)
2. Launch headless browser (Chromium/Firefox/WebKit)
3. For each scenario:
   a. Handle authentication if required (OAuth2/Better Auth)
   b. Navigate to target page
   c. Execute test steps (clicks, inputs, assertions)
   d. Capture artifacts (screenshots, traces, console logs)
   e. Detect issues (rendering errors, broken elements, console errors)
4. Aggregate all issues with context
5. Return structured issue list
```

**Authentication Handling**:
```python
# OAuth2 / Better Auth flow
1. Check if scenario requires authentication
2. If yes:
   a. Use stored credentials from environment variables
   b. Navigate to login page
   c. Fill credentials and submit
   d. Wait for redirect to authenticated state
   e. Store session/cookies in browser context
   f. Reuse context for subsequent tests
3. If no: proceed with unauthenticated browser context
```

**Issue Detection Patterns**:
- **Rendering Errors**: Missing elements, broken layouts, invisible text
- **Console Errors**: JavaScript exceptions, failed network requests
- **Accessibility Violations**: Missing alt text, poor contrast, invalid ARIA
- **Performance Issues**: Slow page loads (>3s), large bundle sizes (>5MB)

**Artifact Capture**:
- Screenshots: Full-page PNG on every test step
- Traces: Playwright trace files for debugging
- Console Logs: All browser console messages (info, warn, error)
- Network Logs: Failed requests with status codes and payloads

**Configuration Example** (`config/frontend-tests.yaml`):
```yaml
scenarios:
  - id: "homepage-load"
    name: "Homepage loads successfully"
    url: "http://localhost:3000"
    auth_required: false
    steps:
      - action: "wait_for_selector"
        selector: "h1"
        timeout: 5000
      - action: "screenshot"
        name: "homepage"
    assertions:
      - selector: "h1"
        text_contains: "Welcome"

  - id: "dashboard-auth"
    name: "Dashboard requires authentication"
    url: "http://localhost:3000/dashboard"
    auth_required: true
    auth_method: "better_auth"
    steps:
      - action: "wait_for_selector"
        selector: ".dashboard"
      - action: "screenshot"
        name: "dashboard"
```

### 3. Backend API Testing (httpx + FastAPI)

**Architecture**:
```python
# API test execution flow
1. Load test scenarios from config (backend-tests.yaml)
2. Initialize httpx AsyncClient
3. For each scenario:
   a. Handle authentication if required (JWT token, session cookie)
   b. Execute HTTP request (GET/POST/PUT/DELETE)
   c. Capture request/response details
   d. Validate response:
      - Status code matches expected
      - Response schema matches OpenAPI spec
      - Performance within thresholds
   e. Detect issues (errors, validation failures, timeouts, slow responses)
4. Aggregate all issues with context
5. Return structured issue list
```

**Authentication Handling**:
```python
# JWT / Session-based auth flow
1. Check if scenario requires authentication
2. If yes:
   a. Use stored credentials from environment variables
   b. Make login request to auth endpoint
   c. Extract token/session from response
   d. Store in client headers/cookies
   e. Reuse for subsequent requests
3. If no: proceed with unauthenticated client
```

**Issue Detection Patterns**:
- **API Errors**: 4xx/5xx status codes, validation errors, timeouts
- **Schema Violations**: Response doesn't match OpenAPI schema
- **Performance Issues**: Response time >1s for p95, >5s for any request
- **Security Issues**: Missing authentication on protected routes, CORS errors

**Artifact Capture**:
- Request Details: Method, URL, headers, body
- Response Details: Status, headers, body, timing
- Error Context: Stack traces for 5xx errors
- Performance Metrics: Response time, payload size

**Configuration Example** (`config/backend-tests.yaml`):
```yaml
scenarios:
  - id: "get-users-list"
    name: "Get users list endpoint"
    method: "GET"
    url: "http://localhost:8000/api/users"
    auth_required: true
    auth_method: "jwt_bearer"
    expected_status: 200
    schema_ref: "#/components/schemas/UserList"
    performance:
      max_response_time_ms: 1000

  - id: "create-user"
    name: "Create new user"
    method: "POST"
    url: "http://localhost:8000/api/users"
    auth_required: true
    body:
      name: "Test User"
      email: "test@example.com"
    expected_status: 201
    schema_ref: "#/components/schemas/User"
```

### 4. Issue Collection & Categorization

**Issue Schema** (Pydantic model):
```python
class IssueType(Enum):
    FRONTEND = "frontend"
    BACKEND = "backend"
    PERFORMANCE = "performance"

class SeverityLevel(Enum):
    CRITICAL = "critical"  # Blocks deployment
    HIGH = "high"          # Significant impact
    MEDIUM = "medium"      # Moderate impact
    LOW = "low"            # Minor impact

class Issue(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    type: IssueType
    severity: SeverityLevel | None = None  # Assigned in analysis phase
    detection_timestamp: datetime
    location: str  # URL/endpoint/file path
    description: str
    captured_context: dict  # Screenshots, request/response, logs
    root_cause: str | None = None  # Assigned in analysis phase
    related_issues: list[str] = []  # Issue IDs, assigned in analysis phase
    resolution_status: str = "pending"  # pending/auto-fixed/manual-required/verified
    resolution_action: str | None = None
```

**Categorization Logic**:
```python
def categorize_issue(raw_issue: dict) -> Issue:
    # Determine type from source
    if raw_issue["source"] == "playwright":
        issue_type = IssueType.FRONTEND
    elif raw_issue["source"] == "httpx":
        issue_type = IssueType.BACKEND
    elif "response_time" in raw_issue:
        issue_type = IssueType.PERFORMANCE

    # Create Issue object
    issue = Issue(
        type=issue_type,
        detection_timestamp=datetime.utcnow(),
        location=raw_issue["location"],
        description=raw_issue["description"],
        captured_context=raw_issue["context"]
    )

    return issue
```

**Storage Format** (`issues.json`):
```json
{
  "workflow_id": "wf-20251217-143022",
  "detection_timestamp": "2025-12-17T14:30:22Z",
  "total_issues": 42,
  "by_type": {
    "frontend": 18,
    "backend": 20,
    "performance": 4
  },
  "issues": [
    {
      "id": "iss-001",
      "type": "frontend",
      "severity": "high",
      "location": "http://localhost:3000/dashboard",
      "description": "Missing alt text on profile image",
      "captured_context": {
        "screenshot": "artifacts/screenshots/dashboard-001.png",
        "selector": "img.profile-avatar",
        "html": "<img src=\"/avatar.jpg\" class=\"profile-avatar\">"
      }
    }
  ]
}
```

### 5. Analysis Layer

**Severity Assignment Algorithm**:
```python
def assign_severity(issue: Issue) -> SeverityLevel:
    """
    Severity assignment based on impact assessment:
    - CRITICAL: Breaks core functionality, security vulnerabilities, data loss
    - HIGH: Significant UX degradation, failed requests, auth issues
    - MEDIUM: Minor UX issues, slow performance, accessibility violations
    - LOW: Cosmetic issues, minor inconsistencies, warnings
    """

    # Critical patterns
    if any(pattern in issue.description.lower() for pattern in [
        "authentication bypass", "sql injection", "xss vulnerability",
        "data corruption", "500 internal server error"
    ]):
        return SeverityLevel.CRITICAL

    # High patterns
    if any(pattern in issue.description.lower() for pattern in [
        "404 not found", "403 forbidden", "broken layout",
        "javascript error", "failed to load"
    ]):
        return SeverityLevel.HIGH

    # Medium patterns
    if any(pattern in issue.description.lower() for pattern in [
        "slow response", "missing alt text", "poor contrast",
        "deprecated api"
    ]):
        return SeverityLevel.MEDIUM

    # Default to low
    return SeverityLevel.LOW
```

**Root Cause Identification**:
```python
def identify_root_cause(issue: Issue, all_issues: list[Issue]) -> str:
    """
    Identify root cause by analyzing:
    1. Timing: First occurrence vs. subsequent
    2. Dependencies: Backend errors causing frontend failures
    3. Patterns: Similar issues across multiple locations
    """

    # Check if this is a symptom of a backend issue
    if issue.type == IssueType.FRONTEND:
        # Look for backend errors with earlier timestamps
        for other in all_issues:
            if (other.type == IssueType.BACKEND and
                other.detection_timestamp < issue.detection_timestamp and
                _are_related(issue, other)):
                return f"Caused by backend issue: {other.id} - {other.description}"

    # Check for pattern across multiple issues
    similar_issues = [i for i in all_issues if _is_similar(issue, i)]
    if len(similar_issues) > 3:
        return f"Systemic issue: {len(similar_issues)} similar occurrences detected"

    # Default: issue is its own root cause
    return "Direct issue - no upstream dependency detected"
```

**Issue Grouping**:
```python
def group_related_issues(issues: list[Issue]) -> list[Issue]:
    """
    Group related issues by:
    1. Same location (URL/endpoint)
    2. Cause-effect relationships (backend → frontend)
    3. Similar descriptions (string similarity)
    """

    for issue in issues:
        # Find issues at same location
        location_matches = [i for i in issues if i.location == issue.location and i.id != issue.id]

        # Find cause-effect relationships
        related_by_cause = [i for i in issues if issue.id in i.root_cause or i.id in issue.root_cause]

        # Find similar descriptions (using fuzzy matching)
        similar_desc = [i for i in issues if _similarity(issue.description, i.description) > 0.8 and i.id != issue.id]

        # Combine and deduplicate
        related = set([i.id for i in location_matches + related_by_cause + similar_desc])
        issue.related_issues = list(related)

    return issues
```

### 6. Resolution Layer

**Auto-Fix Strategy**:
```python
# Safe auto-fix categories:
SAFE_AUTO_FIXES = {
    "missing_alt_text": {
        "pattern": r'<img(?![^>]*alt=)[^>]*>',
        "fix": lambda match: match.group(0).replace('<img', '<img alt=""'),
        "safety": "Low risk - adds accessibility without changing functionality"
    },
    "eslint_formatting": {
        "pattern": "Run ESLint with --fix flag",
        "fix": lambda file: subprocess.run(["eslint", "--fix", file]),
        "safety": "Low risk - formatting only, no logic changes"
    },
    "unused_imports": {
        "pattern": "Unused import detected",
        "fix": lambda file: subprocess.run(["autoflake", "--remove-unused-variables", file]),
        "safety": "Low risk - removes dead code"
    }
}

# Unsafe fixes (require manual intervention):
MANUAL_REQUIRED = {
    "authentication_bypass",
    "sql_injection",
    "logic_error",
    "api_contract_violation",
    "database_migration_required"
}
```

**Fix Application Process**:
```python
def apply_auto_fix(issue: Issue) -> ResolutionAction:
    """
    Apply automated fix with safety guards:
    1. Identify fix pattern
    2. Create backup of affected file
    3. Apply fix
    4. Validate syntax
    5. Run related tests
    6. Rollback if validation fails
    """

    # Check if issue is auto-fixable
    fix_config = _get_fix_config(issue)
    if not fix_config:
        return ResolutionAction(
            issue_id=issue.id,
            action_type="manual_guidance",
            guidance=_generate_guidance(issue),
            outcome="manual_required"
        )

    # Safety guard: backup affected file
    affected_file = _extract_file_path(issue.location)
    backup_path = _create_backup(affected_file)

    try:
        # Apply fix
        _apply_fix_pattern(affected_file, fix_config)

        # Validate syntax
        if not _validate_syntax(affected_file):
            raise ValidationError("Syntax validation failed after fix")

        # Run related tests (if available)
        test_results = _run_related_tests(affected_file)
        if not test_results.passed:
            raise ValidationError("Tests failed after fix")

        # Success
        return ResolutionAction(
            issue_id=issue.id,
            action_type="auto_fix",
            applied_changes=_get_diff(backup_path, affected_file),
            verification_status="passed",
            outcome="fixed"
        )

    except Exception as e:
        # Rollback on any failure
        _restore_backup(backup_path, affected_file)
        return ResolutionAction(
            issue_id=issue.id,
            action_type="auto_fix_failed",
            guidance=f"Auto-fix failed: {str(e)}. Manual intervention required.",
            outcome="rollback"
        )
```

**Guidance Generation for Manual Fixes**:
```python
def generate_resolution_guidance(issue: Issue) -> str:
    """
    Generate step-by-step guidance for manual fixes:
    1. Issue description
    2. Root cause explanation
    3. Recommended solution
    4. Code examples (if applicable)
    5. Testing checklist
    """

    guidance = f"""
    ## Issue: {issue.description}

    **Location**: {issue.location}
    **Severity**: {issue.severity.value}
    **Root Cause**: {issue.root_cause}

    ### Recommended Solution

    {_get_solution_template(issue)}

    ### Testing Checklist

    - [ ] Fix applied in code
    - [ ] Unit tests pass
    - [ ] Integration tests pass
    - [ ] Manual verification completed
    - [ ] No regressions introduced

    ### Related Issues

    {', '.join(issue.related_issues) if issue.related_issues else 'None'}
    """

    return guidance
```

### 7. Re-Testing & Regression Loop

**Re-Test Strategy**:
```python
def retest_after_resolution(
    original_workflow: WorkflowExecution,
    resolution_actions: list[ResolutionAction]
) -> WorkflowExecution:
    """
    Re-run full test suite after resolution:
    1. Re-run all detection tests (frontend + backend + performance)
    2. Compare results with original run
    3. Confirm fixes are applied
    4. Detect any new issues introduced by fixes
    5. Calculate regression score
    """

    # Re-run detection with same configuration
    retest_results = run_detection_phase(original_workflow.config)

    # Compare with original issues
    comparison = compare_issue_sets(
        original=original_workflow.detected_issues,
        retest=retest_results.detected_issues
    )

    # Verify fixed issues are resolved
    fixed_issue_ids = [a.issue_id for a in resolution_actions if a.outcome == "fixed"]
    still_present = [i for i in retest_results.detected_issues if i.id in fixed_issue_ids]

    # Detect regressions (new issues introduced)
    new_issues = [i for i in retest_results.detected_issues if i.id not in
                  [orig.id for orig in original_workflow.detected_issues]]

    # Calculate regression score
    regression_score = len(new_issues) / len(original_workflow.detected_issues) if original_workflow.detected_issues else 0

    return WorkflowExecution(
        id=f"{original_workflow.id}-retest",
        original_workflow_id=original_workflow.id,
        retest_timestamp=datetime.utcnow(),
        detected_issues=retest_results.detected_issues,
        issues_fixed=len(fixed_issue_ids) - len(still_present),
        issues_still_present=len(still_present),
        new_issues_introduced=len(new_issues),
        regression_score=regression_score,
        status="regression_detected" if regression_score > 0.05 else "clean"
    )
```

**Regression Detection**:
```python
def detect_regressions(original: list[Issue], retest: list[Issue]) -> list[Issue]:
    """
    Detect new issues introduced by fixes:
    1. Compare issue signatures (type + location + description)
    2. Flag issues present in retest but not in original
    3. Categorize as regression if caused by recent code changes
    """

    original_signatures = {_issue_signature(i) for i in original}
    regressions = []

    for issue in retest:
        sig = _issue_signature(issue)
        if sig not in original_signatures:
            # New issue detected
            regressions.append(issue)

    return regressions

def _issue_signature(issue: Issue) -> str:
    """Create unique signature for issue comparison"""
    return f"{issue.type.value}:{issue.location}:{issue.description[:100]}"
```

**Loop Termination Criteria**:
```python
MAX_ITERATIONS = 3  # Prevent infinite loops

def should_continue_loop(iteration: int, regression_score: float) -> bool:
    """
    Continue loop if:
    1. Not exceeded max iterations
    2. Regressions detected above threshold
    3. Auto-fixable issues remain
    """

    if iteration >= MAX_ITERATIONS:
        return False

    if regression_score > 0.05:  # 5% new issues is threshold
        return True

    return False
```

### 8. Failure Handling & Safety Guards

**Failure Categories & Handling**:

1. **Environment Failures** (browser crash, network unavailable):
   ```python
   try:
       browser = await playwright.chromium.launch()
   except Exception as e:
       logger.error(f"Browser launch failed: {e}")
       return WorkflowExecution(
           status="environment_failure",
           error_message="Testing environment unavailable. Ensure browser dependencies installed.",
           recovery_action="Run: playwright install"
       )
   ```

2. **Authentication Failures** (invalid credentials, OAuth timeout):
   ```python
   try:
       await authenticate(context, config.auth)
   except AuthenticationError as e:
       logger.error(f"Authentication failed: {e}")
       # Skip auth-required tests, continue with public tests
       return _run_public_tests_only(config)
   ```

3. **Timeout Failures** (test takes too long):
   ```python
   try:
       await page.goto(url, timeout=30000)  # 30s timeout
   except TimeoutError:
       # Log as issue, continue with next test
       issues.append(Issue(
           type=IssueType.PERFORMANCE,
           severity=SeverityLevel.HIGH,
           description=f"Page load timeout: {url}",
           location=url
       ))
   ```

4. **Auto-Fix Failures** (syntax error after fix, tests fail):
   ```python
   # Handled in apply_auto_fix with backup/restore mechanism
   # See Resolution Layer section above
   ```

**Safety Guards**:

1. **Read-Only Detection**: Detection phase never modifies source code
2. **Backup Before Fix**: Always backup files before applying fixes
3. **Syntax Validation**: Validate syntax after every fix
4. **Test Verification**: Run tests after fixes (if available)
5. **Rollback on Failure**: Automatic rollback if any validation fails
6. **Max Iterations**: Limit re-test loops to prevent infinite cycles
7. **Dry-Run Mode**: Optional flag to simulate fixes without applying
8. **Approval Required**: Optional flag to require human approval before fixes

**Configuration** (`config/default.yaml`):
```yaml
safety:
  dry_run: false                 # Simulate fixes without applying
  require_approval: false        # Require human approval before fixes
  backup_enabled: true           # Backup files before fixes
  validate_syntax: true          # Validate syntax after fixes
  run_tests: true                # Run tests after fixes
  max_retest_iterations: 3       # Max re-test loops
  rollback_on_failure: true      # Rollback on validation failure
```

### 9. Output Artifacts

**Report Types**:

1. **JSON Report** (`reports/workflow-{id}.json`):
   ```json
   {
     "workflow_id": "wf-20251217-143022",
     "start_time": "2025-12-17T14:30:22Z",
     "end_time": "2025-12-17T14:58:15Z",
     "duration_seconds": 1673,
     "status": "completed",
     "deployment_ready": true,
     "summary": {
       "total_issues_detected": 42,
       "critical_issues": 2,
       "high_issues": 8,
       "medium_issues": 18,
       "low_issues": 14,
       "auto_fixed": 28,
       "manual_required": 12,
       "regressions_detected": 1
     },
     "phases": {
       "detection": { "duration_seconds": 832, "issues_found": 42 },
       "analysis": { "duration_seconds": 87, "root_causes_identified": 38 },
       "resolution": { "duration_seconds": 241, "fixes_applied": 28 },
       "retest": { "duration_seconds": 498, "regressions_found": 1 },
       "reporting": { "duration_seconds": 15 }
     },
     "issues": [ /* full issue details */ ],
     "resolution_actions": [ /* full resolution details */ ],
     "artifacts": {
       "screenshots": "artifacts/screenshots/",
       "traces": "artifacts/traces/",
       "logs": "artifacts/logs/"
     }
   }
   ```

2. **HTML Dashboard** (`reports/dashboard-{id}.html`):
   - Summary cards (total issues, severity breakdown, deployment status)
   - Issue table (sortable, filterable by type/severity)
   - Timeline visualization (workflow phases)
   - Screenshots gallery (clickable thumbnails)
   - Resolution guidance (expandable sections)

3. **Deployment Readiness Status** (`reports/readiness-{id}.txt`):
   ```
   ╔════════════════════════════════════════╗
   ║   DEPLOYMENT READINESS ASSESSMENT      ║
   ╚════════════════════════════════════════╝

   Status: ✅ READY FOR DEPLOYMENT

   Critical Issues: 0
   High Issues:     0
   Medium Issues:   3 (non-blocking)
   Low Issues:      14 (cosmetic)

   Auto-Fixes Applied:   28
   Manual Fixes Required: 3 (tracked in issue tracker)

   Regressions Detected: 0

   Recommendation: Safe to deploy to production

   Full Report: reports/workflow-wf-20251217-143022.json
   Dashboard:   reports/dashboard-wf-20251217-143022.html
   ```

4. **Log Files** (`artifacts/logs/workflow-{id}.log`):
   ```
   2025-12-17 14:30:22 [INFO] Workflow started: wf-20251217-143022
   2025-12-17 14:30:23 [INFO] Detection phase: Frontend testing started
   2025-12-17 14:30:24 [INFO] Detection phase: Backend testing started
   2025-12-17 14:35:18 [WARN] Frontend test timeout: /dashboard (30s exceeded)
   2025-12-17 14:42:10 [INFO] Detection phase: 42 issues detected
   2025-12-17 14:42:11 [INFO] Analysis phase: Severity assignment started
   ...
   ```

**Artifact Organization**:
```
artifacts/
├── screenshots/
│   └── wf-20251217-143022/
│       ├── homepage-001.png
│       ├── dashboard-002.png
│       └── ...
├── traces/
│   └── wf-20251217-143022/
│       ├── test-001-trace.zip
│       └── ...
├── logs/
│   ├── workflow-wf-20251217-143022.log
│   └── playwright-debug.log
└── backups/
    └── wf-20251217-143022/
        ├── src-app.tsx.bak
        └── ...

reports/
├── workflow-wf-20251217-143022.json
├── dashboard-wf-20251217-143022.html
└── readiness-wf-20251217-143022.txt
```

---

## Integration Points

### CI/CD Integration

**GitHub Actions Example** (`.github/workflows/qa-workflow.yml`):
```yaml
name: Automated QA Workflow

on:
  pull_request:
    branches: [main, develop]
  push:
    branches: [main, develop]

jobs:
  qa-check:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          pip install -e ./qa-automation
          playwright install chromium

      - name: Start application
        run: |
          # Start frontend
          cd AIdd-book && npm start &
          # Start backend
          cd rag-chatbot/backend && uvicorn app.main:app &
          sleep 10  # Wait for services to start

      - name: Run QA workflow
        run: |
          qa-automation run \
            --config qa-automation/config/default.yaml \
            --frontend-url http://localhost:3000 \
            --backend-url http://localhost:8000 \
            --output reports/
        env:
          TEST_AUTH_EMAIL: ${{ secrets.TEST_AUTH_EMAIL }}
          TEST_AUTH_PASSWORD: ${{ secrets.TEST_AUTH_PASSWORD }}

      - name: Check deployment readiness
        run: |
          if [ $(jq -r '.deployment_ready' reports/workflow-*.json) != "true" ]; then
            echo "Deployment NOT ready - critical issues detected"
            exit 1
          fi

      - name: Upload artifacts
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: qa-reports
          path: |
            reports/
            artifacts/
```

### Better Auth Integration

**Authentication Configuration** (no custom auth logic, uses Better Auth endpoints):
```python
# qa-automation/src/services/auth_handler.py

async def authenticate_browser(
    context: BrowserContext,
    auth_config: AuthConfig
) -> None:
    """
    Authenticate browser using Better Auth endpoints.
    No custom auth logic - uses existing Better Auth flow.
    """

    if auth_config.method == "better_auth":
        page = await context.new_page()

        # Navigate to Better Auth login endpoint
        await page.goto(f"{auth_config.base_url}/api/auth/signin")

        # Fill credentials (from environment variables)
        await page.fill('[name="email"]', os.getenv("TEST_AUTH_EMAIL"))
        await page.fill('[name="password"]', os.getenv("TEST_AUTH_PASSWORD"))
        await page.click('button[type="submit"]')

        # Wait for redirect (Better Auth handles session creation)
        await page.wait_for_url(f"{auth_config.base_url}/dashboard")

        # Session now stored in browser context - reuse for tests
        await page.close()

async def authenticate_api(
    client: httpx.AsyncClient,
    auth_config: AuthConfig
) -> None:
    """
    Authenticate API client using Better Auth JWT tokens.
    No custom auth logic - uses existing Better Auth endpoints.
    """

    if auth_config.method == "better_auth":
        # Login via Better Auth endpoint
        response = await client.post(
            f"{auth_config.base_url}/api/auth/signin",
            json={
                "email": os.getenv("TEST_AUTH_EMAIL"),
                "password": os.getenv("TEST_AUTH_PASSWORD")
            }
        )

        # Extract token from response (Better Auth format)
        token = response.json()["access_token"]

        # Add to client headers for subsequent requests
        client.headers["Authorization"] = f"Bearer {token}"
```

---

## Dependencies & Versions

**Core Dependencies** (`pyproject.toml`):
```toml
[project]
name = "qa-automation"
version = "0.1.0"
requires-python = ">=3.11"

dependencies = [
    "playwright>=1.40.0",        # Browser automation
    "httpx>=0.25.0",             # API testing
    "fastapi>=0.115.0",          # Test orchestration API
    "pydantic>=2.5.0",           # Data validation
    "pyyaml>=6.0.1",             # Configuration
    "pytest>=7.4.0",             # Testing framework
    "pytest-asyncio>=0.21.0",    # Async test support
    "rich>=13.7.0",              # CLI output formatting
    "jinja2>=3.1.2",             # HTML report templating
    "jsonschema>=4.20.0",        # Schema validation
]

[project.optional-dependencies]
dev = [
    "pytest-cov>=4.1.0",         # Coverage reports
    "black>=23.12.0",            # Code formatting
    "mypy>=1.7.0",               # Type checking
    "ruff>=0.1.8",               # Linting
]

[project.scripts]
qa-automation = "qa_automation.cli.main:cli"
```

**Installation Commands**:
```bash
# Install QA automation tool
pip install -e ./qa-automation

# Install Playwright browsers
playwright install chromium firefox webkit

# Install development dependencies
pip install -e "./qa-automation[dev]"
```

---

## Next Steps

**Completed in this plan**:
- [x] End-to-end workflow architecture (detect → analyze → resolve → retest → report)
- [x] Frontend browser testing design (Playwright with auth handling)
- [x] Backend API testing design (httpx with FastAPI integration)
- [x] Issue collection and categorization schemas
- [x] Analysis layer (severity, root cause, grouping)
- [x] Resolution layer (auto-fix, guidance, validation)
- [x] Re-testing and regression detection
- [x] Failure handling and safety guards
- [x] Output artifacts and reporting
- [x] CI/CD integration examples
- [x] Better Auth integration (no custom auth logic)

**Ready for Phase 0 (Research)**:
- Research Playwright best practices for SPA testing
- Research httpx async patterns for FastAPI
- Research OpenAPI schema validation libraries
- Research HTML report templating approaches

**Ready for Phase 1 (Design)**:
- Generate `data-model.md` with Pydantic schemas
- Generate `contracts/` with JSON schemas for all entities
- Generate `quickstart.md` with setup and usage guide
- Update agent context with new technologies

**Not in this plan** (will be in `/sp.tasks`):
- Task breakdown for implementation
- Test cases for each component
- Deployment instructions
