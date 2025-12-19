# Data Model: Automated Quality Assurance Workflow

**Date**: 2025-12-17
**Feature**: 001-automated-qa-workflow
**Purpose**: Define all entities, relationships, and validation rules

---

## Overview

The QA workflow system uses five core entities to track the complete lifecycle of automated testing:

1. **Issue** - Represents a detected problem
2. **TestScenario** - Defines a test case to execute
3. **TestResult** - Captures the outcome of a test
4. **ResolutionAction** - Represents an attempted fix
5. **WorkflowExecution** - Tracks a complete workflow run

All entities use Pydantic for data validation and serialization.

---

## Entity Definitions

### 1. Issue

**Purpose**: Represents a detected problem in the application

**Pydantic Model**:
```python
from pydantic import BaseModel, Field
from datetime import datetime
from enum import Enum
from uuid import uuid4

class IssueType(str, Enum):
    """Type of issue detected"""
    FRONTEND = "frontend"
    BACKEND = "backend"
    PERFORMANCE = "performance"

class SeverityLevel(str, Enum):
    """Severity classification"""
    CRITICAL = "critical"  # Blocks deployment
    HIGH = "high"          # Significant impact
    MEDIUM = "medium"      # Moderate impact
    LOW = "low"            # Minor impact

class ResolutionStatus(str, Enum):
    """Current resolution state"""
    PENDING = "pending"
    AUTO_FIXED = "auto_fixed"
    MANUAL_REQUIRED = "manual_required"
    VERIFIED = "verified"
    WONT_FIX = "wont_fix"

class Issue(BaseModel):
    """Issue entity"""
    id: str = Field(default_factory=lambda: str(uuid4()))
    type: IssueType
    severity: SeverityLevel | None = None  # Assigned in analysis phase
    detection_timestamp: datetime = Field(default_factory=datetime.utcnow)
    location: str = Field(..., description="URL/endpoint/file path where issue was detected")
    description: str = Field(..., min_length=10, max_length=1000)
    captured_context: dict = Field(default_factory=dict, description="Screenshots, request/response data, logs")
    root_cause: str | None = Field(None, description="Identified root cause (assigned in analysis)")
    related_issues: list[str] = Field(default_factory=list, description="List of related issue IDs")
    resolution_status: ResolutionStatus = ResolutionStatus.PENDING
    resolution_action_id: str | None = Field(None, description="ID of resolution action taken")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "iss-001",
                "type": "frontend",
                "severity": "high",
                "detection_timestamp": "2025-12-17T14:30:22Z",
                "location": "http://localhost:3000/dashboard",
                "description": "Missing alt text on profile image",
                "captured_context": {
                    "screenshot": "artifacts/screenshots/dashboard-001.png",
                    "selector": "img.profile-avatar",
                    "html": "<img src='/avatar.jpg' class='profile-avatar'>"
                },
                "root_cause": "Direct issue - no upstream dependency detected",
                "related_issues": ["iss-002", "iss-003"],
                "resolution_status": "pending",
                "resolution_action_id": null
            }
        }
```

**Validation Rules**:
- `id`: Must be unique UUID string
- `type`: Must be one of: frontend, backend, performance
- `severity`: Optional during detection, required after analysis
- `description`: Between 10-1000 characters
- `location`: Must be a valid URL, endpoint path, or file path
- `captured_context`: Dictionary with artifact references (screenshots, traces, logs)
- `related_issues`: List of valid issue IDs

**Relationships**:
- One Issue → Zero or Many related Issues (self-referential)
- One Issue → Zero or One ResolutionAction

---

### 2. TestScenario

**Purpose**: Defines a specific test case to execute

**Pydantic Model**:
```python
class TestType(str, Enum):
    """Type of test scenario"""
    FRONTEND = "frontend"
    BACKEND = "backend"
    PERFORMANCE = "performance"

class AuthMethod(str, Enum):
    """Authentication method for test"""
    NONE = "none"
    BETTER_AUTH = "better_auth"
    OAUTH2 = "oauth2"
    JWT_BEARER = "jwt_bearer"
    SESSION_COOKIE = "session_cookie"

class TestStep(BaseModel):
    """Individual step in a test scenario"""
    action: str = Field(..., description="Action type: wait_for_selector, click, fill, screenshot, etc.")
    selector: str | None = Field(None, description="CSS selector for element")
    value: str | None = Field(None, description="Value for fill actions")
    timeout: int = Field(5000, description="Timeout in milliseconds")

class Assertion(BaseModel):
    """Assertion to verify in test"""
    selector: str | None = None
    text_contains: str | None = None
    status_code: int | None = None
    response_schema: str | None = None

class TestScenario(BaseModel):
    """Test scenario entity"""
    id: str = Field(..., description="Unique scenario identifier")
    name: str = Field(..., min_length=5, max_length=200)
    type: TestType
    enabled: bool = Field(True, description="Whether scenario is enabled")
    url: str | None = Field(None, description="URL for frontend tests or API endpoint for backend tests")
    method: str | None = Field(None, description="HTTP method for backend tests (GET, POST, etc.)")
    auth_required: bool = Field(False)
    auth_method: AuthMethod = AuthMethod.NONE
    headers: dict[str, str] = Field(default_factory=dict)
    body: dict | None = Field(None, description="Request body for backend tests")
    steps: list[TestStep] = Field(default_factory=list, description="Steps for frontend tests")
    assertions: list[Assertion] = Field(default_factory=list)
    performance_thresholds: dict = Field(default_factory=dict, description="Max response time, bundle size, etc.")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "homepage-load",
                "name": "Homepage loads successfully",
                "type": "frontend",
                "enabled": true,
                "url": "http://localhost:3000",
                "method": null,
                "auth_required": false,
                "auth_method": "none",
                "steps": [
                    {
                        "action": "wait_for_selector",
                        "selector": "h1",
                        "timeout": 5000
                    },
                    {
                        "action": "screenshot",
                        "value": "homepage"
                    }
                ],
                "assertions": [
                    {
                        "selector": "h1",
                        "text_contains": "Welcome"
                    }
                ],
                "performance_thresholds": {
                    "max_page_load_ms": 3000
                }
            }
        }
```

**Validation Rules**:
- `id`: Unique string identifier (kebab-case recommended)
- `name`: Between 5-200 characters
- `type`: Must be one of: frontend, backend, performance
- `url`: Required for frontend tests and backend tests
- `method`: Required for backend tests (GET, POST, PUT, DELETE, PATCH)
- `auth_method`: Must match auth_required (if auth_required=true, method != none)
- `steps`: Required for frontend tests, empty for backend tests
- `assertions`: At least one assertion required

**Relationships**:
- One TestScenario → Many TestResults (historical executions)

---

### 3. TestResult

**Purpose**: Captures the outcome of executing a test scenario

**Pydantic Model**:
```python
class TestStatus(str, Enum):
    """Test execution status"""
    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    SKIPPED = "skipped"
    TIMEOUT = "timeout"

class TestResult(BaseModel):
    """Test result entity"""
    id: str = Field(default_factory=lambda: str(uuid4()))
    scenario_id: str = Field(..., description="Reference to TestScenario.id")
    workflow_execution_id: str = Field(..., description="Reference to WorkflowExecution.id")
    execution_timestamp: datetime = Field(default_factory=datetime.utcnow)
    status: TestStatus
    duration_ms: int = Field(..., ge=0, description="Execution time in milliseconds")
    error_message: str | None = Field(None, max_length=2000)
    detected_issues: list[str] = Field(default_factory=list, description="List of Issue.id detected by this test")
    performance_metrics: dict = Field(default_factory=dict, description="Response time, bundle size, etc.")
    captured_artifacts: dict = Field(default_factory=dict, description="Screenshots, traces, logs")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "tr-001",
                "scenario_id": "homepage-load",
                "workflow_execution_id": "wf-20251217-143022",
                "execution_timestamp": "2025-12-17T14:30:25Z",
                "status": "passed",
                "duration_ms": 1834,
                "error_message": null,
                "detected_issues": [],
                "performance_metrics": {
                    "page_load_ms": 1834,
                    "bundle_size_kb": 285,
                    "num_requests": 12
                },
                "captured_artifacts": {
                    "screenshot": "artifacts/screenshots/wf-20251217-143022/homepage-001.png",
                    "trace": "artifacts/traces/wf-20251217-143022/test-001-trace.zip"
                }
            }
        }
```

**Validation Rules**:
- `id`: Unique UUID string
- `scenario_id`: Must reference a valid TestScenario
- `workflow_execution_id`: Must reference a valid WorkflowExecution
- `status`: Must be one of: passed, failed, error, skipped, timeout
- `duration_ms`: Non-negative integer
- `error_message`: Optional, max 2000 characters
- `detected_issues`: List of valid Issue IDs
- `performance_metrics`: Dictionary with numeric values
- `captured_artifacts`: Dictionary with file path references

**Relationships**:
- One TestResult → One TestScenario
- One TestResult → One WorkflowExecution
- One TestResult → Many Issues (detected during this test)

---

### 4. ResolutionAction

**Purpose**: Represents an attempted fix for an issue

**Pydantic Model**:
```python
class ActionType(str, Enum):
    """Type of resolution action"""
    AUTO_FIX = "auto_fix"
    AUTO_FIX_FAILED = "auto_fix_failed"
    MANUAL_GUIDANCE = "manual_guidance"

class VerificationStatus(str, Enum):
    """Result of fix verification"""
    PASSED = "passed"
    FAILED = "failed"
    NOT_VERIFIED = "not_verified"

class ResolutionOutcome(str, Enum):
    """Final outcome of resolution"""
    FIXED = "fixed"
    MANUAL_REQUIRED = "manual_required"
    ROLLBACK = "rollback"

class ResolutionAction(BaseModel):
    """Resolution action entity"""
    id: str = Field(default_factory=lambda: str(uuid4()))
    issue_id: str = Field(..., description="Reference to Issue.id")
    workflow_execution_id: str = Field(..., description="Reference to WorkflowExecution.id")
    action_timestamp: datetime = Field(default_factory=datetime.utcnow)
    action_type: ActionType
    applied_changes: str | None = Field(None, max_length=10000, description="Git diff or description of changes")
    guidance: str | None = Field(None, max_length=5000, description="Manual resolution guidance")
    verification_status: VerificationStatus = VerificationStatus.NOT_VERIFIED
    outcome: ResolutionOutcome

    class Config:
        json_schema_extra = {
            "example": {
                "id": "ra-001",
                "issue_id": "iss-001",
                "workflow_execution_id": "wf-20251217-143022",
                "action_timestamp": "2025-12-17T14:45:12Z",
                "action_type": "auto_fix",
                "applied_changes": "--- a/src/components/ProfileImage.tsx\n+++ b/src/components/ProfileImage.tsx\n@@ -10,7 +10,7 @@\n-  <img src={avatarUrl} className=\"profile-avatar\" />\n+  <img src={avatarUrl} alt=\"User profile avatar\" className=\"profile-avatar\" />",
                "guidance": null,
                "verification_status": "passed",
                "outcome": "fixed"
            }
        }
```

**Validation Rules**:
- `id`: Unique UUID string
- `issue_id`: Must reference a valid Issue
- `workflow_execution_id`: Must reference a valid WorkflowExecution
- `action_type`: Must be one of: auto_fix, auto_fix_failed, manual_guidance
- `applied_changes`: Optional, max 10000 characters (large diffs)
- `guidance`: Optional, max 5000 characters
- `verification_status`: Must be one of: passed, failed, not_verified
- `outcome`: Must be one of: fixed, manual_required, rollback
- **Constraint**: If action_type == "auto_fix", applied_changes must not be null
- **Constraint**: If action_type == "manual_guidance", guidance must not be null

**Relationships**:
- One ResolutionAction → One Issue
- One ResolutionAction → One WorkflowExecution

---

### 5. WorkflowExecution

**Purpose**: Tracks a complete run of the QA workflow

**Pydantic Model**:
```python
class WorkflowStatus(str, Enum):
    """Overall workflow status"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

class PhaseResult(BaseModel):
    """Result of a workflow phase"""
    phase_name: str
    duration_seconds: int = Field(..., ge=0)
    status: str
    items_processed: int = Field(..., ge=0)
    items_successful: int = Field(..., ge=0)
    items_failed: int = Field(..., ge=0)

class WorkflowSummary(BaseModel):
    """Summary statistics for workflow"""
    total_issues_detected: int = Field(..., ge=0)
    critical_issues: int = Field(..., ge=0)
    high_issues: int = Field(..., ge=0)
    medium_issues: int = Field(..., ge=0)
    low_issues: int = Field(..., ge=0)
    auto_fixed: int = Field(..., ge=0)
    manual_required: int = Field(..., ge=0)
    regressions_detected: int = Field(..., ge=0)

class WorkflowExecution(BaseModel):
    """Workflow execution entity"""
    id: str = Field(default_factory=lambda: f"wf-{datetime.utcnow().strftime('%Y%m%d-%H%M%S')}")
    start_time: datetime = Field(default_factory=datetime.utcnow)
    end_time: datetime | None = None
    duration_seconds: int | None = Field(None, ge=0)
    status: WorkflowStatus = WorkflowStatus.PENDING
    config: dict = Field(..., description="Configuration used for this run")
    deployment_ready: bool | None = Field(None, description="Whether app is ready for deployment")

    # Phase results
    phase_results: list[PhaseResult] = Field(default_factory=list)

    # Summary
    summary: WorkflowSummary | None = None

    # References
    test_results: list[str] = Field(default_factory=list, description="List of TestResult.id")
    detected_issues: list[str] = Field(default_factory=list, description="List of Issue.id")
    resolution_actions: list[str] = Field(default_factory=list, description="List of ResolutionAction.id")

    # Artifacts
    artifacts: dict = Field(default_factory=dict, description="Paths to screenshots, traces, logs, reports")

    # Re-test tracking
    original_workflow_id: str | None = Field(None, description="If this is a re-test, ID of original workflow")
    retest_iteration: int = Field(0, description="Re-test iteration number (0 = initial run)")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "wf-20251217-143022",
                "start_time": "2025-12-17T14:30:22Z",
                "end_time": "2025-12-17T14:58:15Z",
                "duration_seconds": 1673,
                "status": "completed",
                "deployment_ready": true,
                "config": {
                    "frontend_url": "http://localhost:3000",
                    "backend_url": "http://localhost:8000",
                    "max_retest_iterations": 3
                },
                "phase_results": [
                    {
                        "phase_name": "detection",
                        "duration_seconds": 832,
                        "status": "completed",
                        "items_processed": 150,
                        "items_successful": 148,
                        "items_failed": 2
                    }
                ],
                "summary": {
                    "total_issues_detected": 42,
                    "critical_issues": 0,
                    "high_issues": 8,
                    "medium_issues": 18,
                    "low_issues": 16,
                    "auto_fixed": 28,
                    "manual_required": 12,
                    "regressions_detected": 1
                },
                "test_results": ["tr-001", "tr-002"],
                "detected_issues": ["iss-001", "iss-002"],
                "resolution_actions": ["ra-001"],
                "artifacts": {
                    "screenshots": "artifacts/screenshots/wf-20251217-143022/",
                    "traces": "artifacts/traces/wf-20251217-143022/",
                    "logs": "artifacts/logs/workflow-wf-20251217-143022.log",
                    "report_json": "reports/workflow-wf-20251217-143022.json",
                    "report_html": "reports/dashboard-wf-20251217-143022.html"
                },
                "original_workflow_id": null,
                "retest_iteration": 0
            }
        }
```

**Validation Rules**:
- `id`: Unique workflow identifier (timestamp-based)
- `start_time`: Always set to creation time
- `end_time`: Set when workflow completes
- `duration_seconds`: Calculated as (end_time - start_time)
- `status`: Must be one of: pending, running, completed, failed, cancelled
- `deployment_ready`: true only if critical_issues == 0 and high_issues == 0
- `config`: Dictionary with workflow configuration
- `phase_results`: List of results for each phase (detection, analysis, resolution, retest, reporting)
- `summary`: Summary statistics, calculated after analysis phase
- `test_results`: List of TestResult IDs from this workflow
- `detected_issues`: List of Issue IDs detected in this workflow
- `resolution_actions`: List of ResolutionAction IDs from this workflow
- `retest_iteration`: 0 for initial run, 1+ for re-tests

**Relationships**:
- One WorkflowExecution → Many TestResults
- One WorkflowExecution → Many Issues
- One WorkflowExecution → Many ResolutionActions
- One WorkflowExecution → Zero or One parent WorkflowExecution (for re-tests)

---

## Entity Relationship Diagram

```
┌─────────────────────┐
│ WorkflowExecution   │
│ ─────────────────── │
│ id (PK)             │
│ start_time          │
│ end_time            │
│ status              │
│ deployment_ready    │
│ test_results[]      │◄─────┐
│ detected_issues[]   │◄─────┼─────┐
│ resolution_actions[]│◄─────┼─────┼─────┐
└─────────────────────┘      │     │     │
                             │     │     │
                ┌────────────┴─────┴─────┴──────────┐
                │                                    │
┌───────────────▼───────┐    ┌─────────────────────▼──┐    ┌─────────────────────▼──┐
│ TestResult            │    │ Issue                  │    │ ResolutionAction       │
│ ───────────────────── │    │ ──────────────────────  │    │ ──────────────────────  │
│ id (PK)               │    │ id (PK)                │    │ id (PK)                │
│ scenario_id (FK)      │───▶│ type                   │◄───│ issue_id (FK)          │
│ workflow_execution_id │    │ severity               │    │ workflow_execution_id  │
│ execution_timestamp   │    │ location               │    │ action_timestamp       │
│ status                │    │ description            │    │ action_type            │
│ duration_ms           │    │ captured_context       │    │ applied_changes        │
│ detected_issues[]     │───▶│ root_cause             │    │ guidance               │
│ performance_metrics   │    │ related_issues[]       │────│ verification_status    │
│ captured_artifacts    │    │ resolution_status      │    │ outcome                │
└───────────────────────┘    │ resolution_action_id   │────└────────────────────────┘
                │            └────────────────────────┘
                │                      │
                │                      │ (self-referential)
                │                      │ related_issues
                │                      │
                │            ┌─────────▼──────────┐
                │            │ Issue              │
                └───────────▶│ (related issues)   │
                             └────────────────────┘
```

---

## Data Validation Examples

### Valid Issue
```python
issue = Issue(
    type=IssueType.FRONTEND,
    severity=SeverityLevel.HIGH,
    location="http://localhost:3000/dashboard",
    description="Missing alt text on profile image - accessibility violation",
    captured_context={
        "screenshot": "artifacts/screenshots/dashboard-001.png",
        "selector": "img.profile-avatar"
    }
)
# ✅ Valid
```

### Invalid Issue (description too short)
```python
issue = Issue(
    type=IssueType.FRONTEND,
    location="http://localhost:3000",
    description="Error"  # Too short (<10 chars)
)
# ❌ ValidationError: description must be at least 10 characters
```

### Valid TestScenario (Frontend)
```python
scenario = TestScenario(
    id="dashboard-auth",
    name="Dashboard requires authentication",
    type=TestType.FRONTEND,
    url="http://localhost:3000/dashboard",
    auth_required=True,
    auth_method=AuthMethod.BETTER_AUTH,
    steps=[
        TestStep(action="wait_for_selector", selector=".dashboard", timeout=5000),
        TestStep(action="screenshot", value="dashboard")
    ],
    assertions=[
        Assertion(selector=".dashboard", text_contains="Welcome")
    ]
)
# ✅ Valid
```

### Invalid TestScenario (auth mismatch)
```python
scenario = TestScenario(
    id="test-1",
    name="Test",
    type=TestType.FRONTEND,
    url="http://localhost:3000",
    auth_required=True,
    auth_method=AuthMethod.NONE  # Mismatch: auth required but method is none
)
# ❌ ValidationError: auth_method must not be 'none' when auth_required is True
```

---

## Storage & Serialization

### JSON Serialization
All entities serialize to JSON using Pydantic's `model_dump_json()`:

```python
issue = Issue(...)
json_str = issue.model_dump_json(indent=2)
# Produces valid JSON with ISO 8601 timestamps

# Deserialize
issue = Issue.model_validate_json(json_str)
```

### File Storage
Entities are stored in JSON files organized by type:

```
data/
├── workflows/
│   └── wf-20251217-143022.json
├── test-results/
│   ├── tr-001.json
│   └── tr-002.json
├── issues/
│   ├── iss-001.json
│   └── iss-002.json
└── resolution-actions/
    └── ra-001.json
```

### Database Schema (Optional Future Enhancement)
For production deployment, entities can be stored in PostgreSQL:

```sql
CREATE TABLE workflow_executions (
    id VARCHAR(50) PRIMARY KEY,
    start_time TIMESTAMP NOT NULL,
    end_time TIMESTAMP,
    duration_seconds INTEGER,
    status VARCHAR(20) NOT NULL,
    config JSONB NOT NULL,
    deployment_ready BOOLEAN,
    summary JSONB,
    artifacts JSONB,
    original_workflow_id VARCHAR(50),
    retest_iteration INTEGER DEFAULT 0,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE issues (
    id VARCHAR(50) PRIMARY KEY,
    type VARCHAR(20) NOT NULL,
    severity VARCHAR(20),
    detection_timestamp TIMESTAMP NOT NULL,
    location TEXT NOT NULL,
    description TEXT NOT NULL,
    captured_context JSONB,
    root_cause TEXT,
    related_issues JSONB,
    resolution_status VARCHAR(20) NOT NULL,
    resolution_action_id VARCHAR(50),
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE test_results (
    id VARCHAR(50) PRIMARY KEY,
    scenario_id VARCHAR(100) NOT NULL,
    workflow_execution_id VARCHAR(50) REFERENCES workflow_executions(id),
    execution_timestamp TIMESTAMP NOT NULL,
    status VARCHAR(20) NOT NULL,
    duration_ms INTEGER NOT NULL,
    error_message TEXT,
    detected_issues JSONB,
    performance_metrics JSONB,
    captured_artifacts JSONB,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE resolution_actions (
    id VARCHAR(50) PRIMARY KEY,
    issue_id VARCHAR(50) REFERENCES issues(id),
    workflow_execution_id VARCHAR(50) REFERENCES workflow_executions(id),
    action_timestamp TIMESTAMP NOT NULL,
    action_type VARCHAR(30) NOT NULL,
    applied_changes TEXT,
    guidance TEXT,
    verification_status VARCHAR(20) NOT NULL,
    outcome VARCHAR(20) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);
```

---

## Summary

All entities are fully defined with:
- ✅ Pydantic models with validation
- ✅ Enums for type safety
- ✅ Field constraints and descriptions
- ✅ Example JSON for each entity
- ✅ Relationship mappings
- ✅ Serialization/deserialization patterns
- ✅ Storage strategies (JSON files + optional DB schema)

**Ready for implementation and contract generation**.
