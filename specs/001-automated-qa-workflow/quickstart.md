# Quickstart Guide: Automated Quality Assurance Workflow

**Date**: 2025-12-17
**Feature**: 001-automated-qa-workflow
**Purpose**: Setup and usage guide for QA automation system

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Running Your First Workflow](#running-your-first-workflow)
5. [Understanding Results](#understanding-results)
6. [Advanced Usage](#advanced-usage)
7. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### System Requirements
- **Python**: 3.11 or higher
- **Operating System**: Linux or macOS (Windows WSL2 supported)
- **Memory**: Minimum 4GB RAM (8GB+ recommended for parallel testing)
- **Disk Space**: 500MB for dependencies + space for artifacts

### Application Requirements
- Frontend application running locally or on accessible server
- Backend API running locally or on accessible server
- Test credentials for authenticated routes (if applicable)

---

## Installation

### Step 1: Install QA Automation Package

```bash
# Clone the repository (or navigate to project root)
cd /path/to/project

# Install the QA automation package
pip install -e ./qa-automation

# Verify installation
qa-automation --version
```

### Step 2: Install Browser Dependencies

```bash
# Install Playwright browsers (required for frontend testing)
playwright install chromium

# Optional: Install Firefox and WebKit for multi-browser testing
playwright install firefox webkit
```

### Step 3: Verify Installation

```bash
# Run installation check
qa-automation check

# Expected output:
# ✅ Python version: 3.11.x
# ✅ Playwright installed
# ✅ Chromium browser available
# ✅ Configuration directory: qa-automation/config/
```

---

## Configuration

### Step 1: Set Up Environment Variables

Create a `.env` file in your project root:

```bash
# Authentication credentials for testing
TEST_AUTH_EMAIL=test.user@example.com
TEST_AUTH_PASSWORD=SecureTestPassword123!

# Optional: Custom configuration paths
QA_CONFIG_DIR=qa-automation/config/
QA_ARTIFACTS_DIR=artifacts/
QA_REPORTS_DIR=reports/
```

**Important**: Never commit `.env` files with real credentials!

### Step 2: Configure Workflow

Edit `qa-automation/config/default.yaml`:

```yaml
# Frontend and backend URLs
frontend_url: http://localhost:3000
backend_url: http://localhost:8000

# Browser settings
browser: chromium
headless: true
parallel: true
max_concurrent: 10
timeout: 30000

# Test scenarios
scenarios:
  frontend: config/frontend-tests.yaml
  backend: config/backend-tests.yaml

# Authentication
auth:
  email: ${TEST_AUTH_EMAIL}
  password: ${TEST_AUTH_PASSWORD}

# Safety settings
safety:
  dry_run: false              # Set to true to simulate fixes
  require_approval: false     # Set to true for manual approval
  backup_enabled: true
  validate_syntax: true
  run_tests: true
  max_retest_iterations: 3
  rollback_on_failure: true

# Output settings
output:
  artifacts_dir: artifacts
  reports_dir: reports
  generate_html: true
  generate_json: true
```

### Step 3: Define Test Scenarios

#### Frontend Tests (`config/frontend-tests.yaml`)

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
    performance_thresholds:
      max_page_load_ms: 3000

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
    assertions:
      - selector: ".dashboard"
```

#### Backend Tests (`config/backend-tests.yaml`)

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

  - id: "health-check"
    name: "Health check endpoint"
    method: "GET"
    url: "http://localhost:8000/health"
    auth_required: false
    expected_status: 200
```

---

## Running Your First Workflow

### Basic Usage

```bash
# Run complete workflow
qa-automation run

# Expected output:
# 🚀 Starting QA Workflow (wf-20251217-143022)
#
# Phase 1/5: Detection
# ├─ Frontend testing... 148/150 tests passed
# ├─ Backend testing... 98/100 tests passed
# └─ 42 issues detected
#
# Phase 2/5: Analysis
# ├─ Severity assignment... 42/42 complete
# ├─ Root cause analysis... 38/42 identified
# └─ Issue grouping... 12 groups created
#
# Phase 3/5: Resolution
# ├─ Auto-fixes attempted... 28/30 successful
# └─ Manual guidance generated for 12 issues
#
# Phase 4/5: Re-testing
# ├─ Regression testing... 1 new issue detected
# └─ Fixed issues verified: 26/28 confirmed
#
# Phase 5/5: Reporting
# └─ Reports generated
#
# ✅ Workflow completed in 27m 53s
# 📊 Deployment ready: YES
# 📁 Reports: reports/workflow-wf-20251217-143022.json
# 🌐 Dashboard: reports/dashboard-wf-20251217-143022.html
```

### Dry Run Mode

Test the workflow without applying fixes:

```bash
qa-automation run --dry-run

# Simulates all phases but doesn't modify any files
# Useful for understanding what would be changed
```

### Custom Configuration

```bash
# Use custom config file
qa-automation run --config path/to/custom-config.yaml

# Override URLs
qa-automation run \
  --frontend-url http://staging.example.com \
  --backend-url http://api.staging.example.com
```

### Specific Phases

Run individual phases for debugging:

```bash
# Detection only
qa-automation detect --output issues.json

# Analysis only (requires existing issues.json)
qa-automation analyze --input issues.json --output analyzed-issues.json

# Resolution only
qa-automation resolve --input analyzed-issues.json --output resolution-actions.json
```

---

## Understanding Results

### JSON Report

Location: `reports/workflow-{id}.json`

```json
{
  "workflow_id": "wf-20251217-143022",
  "status": "completed",
  "deployment_ready": true,
  "summary": {
    "total_issues_detected": 42,
    "critical_issues": 0,
    "high_issues": 8,
    "medium_issues": 18,
    "low_issues": 16,
    "auto_fixed": 28,
    "manual_required": 12
  }
}
```

**Key Fields**:
- `deployment_ready`: `true` = safe to deploy, `false` = critical/high issues remain
- `summary.critical_issues`: Must be 0 for deployment
- `summary.high_issues`: Should be 0 for deployment
- `summary.auto_fixed`: Issues automatically resolved
- `summary.manual_required`: Issues requiring manual intervention

### HTML Dashboard

Location: `reports/dashboard-{id}.html`

**Sections**:
1. **Summary Cards**: Quick overview of issues by severity
2. **Issue Table**: Sortable, filterable list of all issues
3. **Timeline**: Visual representation of workflow phases
4. **Screenshots**: Gallery of captured screenshots
5. **Resolution Guidance**: Expandable sections with fix instructions

Open in browser:
```bash
open reports/dashboard-wf-20251217-143022.html
```

### Deployment Readiness Status

Location: `reports/readiness-{id}.txt`

```
╔════════════════════════════════════════╗
║   DEPLOYMENT READINESS ASSESSMENT      ║
╚════════════════════════════════════════╝

Status: ✅ READY FOR DEPLOYMENT

Critical Issues: 0
High Issues:     0
Medium Issues:   3 (non-blocking)
Low Issues:      14 (cosmetic)

Recommendation: Safe to deploy to production
```

**Deployment Criteria**:
- ✅ **Ready**: `critical_issues == 0` AND `high_issues == 0`
- ⚠️  **Caution**: `critical_issues == 0` AND `high_issues > 0` (review high issues first)
- ❌ **Not Ready**: `critical_issues > 0` (must fix before deployment)

### Artifacts

#### Screenshots
Location: `artifacts/screenshots/{workflow-id}/`

- Full-page PNG images captured during tests
- Organized by test scenario
- Useful for debugging UI issues

#### Traces
Location: `artifacts/traces/{workflow-id}/`

- Playwright trace files (`.zip` format)
- Open with: `playwright show-trace artifacts/traces/.../trace.zip`
- Contains:
  - Network activity
  - Console logs
  - DOM snapshots
  - Timeline of actions

#### Logs
Location: `artifacts/logs/workflow-{id}.log`

- Detailed execution logs
- Includes timestamps, severity levels, and context
- Search for errors: `grep ERROR artifacts/logs/workflow-*.log`

---

## Advanced Usage

### CI/CD Integration

#### GitHub Actions

```yaml
# .github/workflows/qa-check.yml
name: QA Workflow Check

on:
  pull_request:
    branches: [main, develop]

jobs:
  qa-check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          pip install -e ./qa-automation
          playwright install chromium

      - name: Start services
        run: |
          docker-compose up -d
          sleep 10

      - name: Run QA workflow
        run: qa-automation run
        env:
          TEST_AUTH_EMAIL: ${{ secrets.TEST_AUTH_EMAIL }}
          TEST_AUTH_PASSWORD: ${{ secrets.TEST_AUTH_PASSWORD }}

      - name: Check deployment readiness
        run: |
          if ! jq -e '.deployment_ready == true' reports/workflow-*.json; then
            echo "Deployment NOT ready"
            exit 1
          fi

      - name: Upload reports
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: qa-reports
          path: reports/
```

### Programmatic API

Use the workflow as a Python library:

```python
from qa_automation.lib.workflow import WorkflowEngine
from qa_automation.models.workflow import WorkflowConfig

# Load configuration
config = WorkflowConfig.from_yaml("config/default.yaml")

# Create workflow engine
engine = WorkflowEngine(config)

# Run workflow
result = await engine.run()

# Check deployment readiness
if result.deployment_ready:
    print("✅ Ready for deployment")
else:
    print(f"❌ Not ready: {result.summary.critical_issues} critical issues")

# Access issues
for issue in result.detected_issues:
    print(f"- {issue.severity}: {issue.description}")
```

### Custom Scenarios via API

```python
from qa_automation.models.test_scenario import TestScenario, TestStep, Assertion

# Define custom scenario
scenario = TestScenario(
    id="custom-checkout-flow",
    name="Complete checkout process",
    type="frontend",
    url="http://localhost:3000/cart",
    auth_required=True,
    auth_method="better_auth",
    steps=[
        TestStep(action="wait_for_selector", selector=".cart-items"),
        TestStep(action="click", selector="button.checkout"),
        TestStep(action="wait_for_selector", selector=".payment-form"),
        TestStep(action="fill", selector="[name='card-number']", value="4242424242424242"),
        TestStep(action="click", selector="button.submit-payment"),
        TestStep(action="wait_for_selector", selector=".order-confirmation"),
        TestStep(action="screenshot", value="checkout-complete")
    ],
    assertions=[
        Assertion(selector=".order-confirmation", text_contains="Order confirmed")
    ]
)

# Run single scenario
from qa_automation.services.detector import FrontendDetector

detector = FrontendDetector(config)
result = await detector.run_scenario(scenario)
```

---

## Troubleshooting

### Common Issues

#### 1. Browser Launch Failed

**Error**: `playwright._impl._api_types.Error: Executable doesn't exist`

**Solution**:
```bash
playwright install chromium
```

#### 2. Authentication Failed

**Error**: `AuthenticationError: Failed to authenticate with Better Auth`

**Checklist**:
- ✅ Verify `TEST_AUTH_EMAIL` and `TEST_AUTH_PASSWORD` in `.env`
- ✅ Ensure auth endpoint is correct in config
- ✅ Check that test user exists in database
- ✅ Verify application is running and accessible

#### 3. Timeout Errors

**Error**: `TimeoutError: page.goto: Timeout 30000ms exceeded`

**Solutions**:
- Increase timeout in config: `timeout: 60000`
- Check if application is responsive
- Verify network connectivity
- Check browser console logs in traces

#### 4. Issues Not Detected

**Problem**: Workflow completes but no issues found (when you know issues exist)

**Checklist**:
- ✅ Verify test scenarios cover the problematic areas
- ✅ Check selectors are correct (use browser DevTools)
- ✅ Review logs for skipped tests
- ✅ Ensure assertions are properly configured

#### 5. Auto-Fix Failed

**Error**: `AutoFixError: Syntax validation failed after fix`

**What happened**: The automated fix introduced a syntax error and was rolled back

**Next steps**:
- Check logs for details: `grep "auto_fix_failed" artifacts/logs/workflow-*.log`
- Review manual guidance in HTML report
- Fix issue manually and re-run workflow

### Debug Mode

Enable verbose logging:

```bash
# Set log level to DEBUG
QA_LOG_LEVEL=DEBUG qa-automation run

# Save debug logs to file
QA_LOG_LEVEL=DEBUG qa-automation run 2>&1 | tee debug.log
```

### Testing Scenarios Individually

Test a single scenario before running full workflow:

```bash
# Test specific frontend scenario
qa-automation test-scenario --type frontend --id homepage-load

# Test specific backend scenario
qa-automation test-scenario --type backend --id health-check
```

---

## Next Steps

After completing this quickstart:

1. **Customize Test Scenarios**: Add scenarios specific to your application
2. **Integrate with CI/CD**: Automate QA checks on every pull request
3. **Tune Safety Settings**: Adjust auto-fix categories based on your comfort level
4. **Monitor Trends**: Track issue counts and deployment readiness over time
5. **Extend with Custom Analyzers**: Add domain-specific issue detection logic

**Documentation**:
- [Data Model Reference](./data-model.md)
- [Configuration Schema](./contracts/workflow-config-schema.json)
- [API Documentation](../../qa-automation/README.md)

**Support**:
- Report issues: [GitHub Issues](https://github.com/your-org/project/issues)
- Ask questions: [Discussions](https://github.com/your-org/project/discussions)
