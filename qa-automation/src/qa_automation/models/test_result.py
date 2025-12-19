"""Test result entity model."""

from datetime import datetime
from enum import Enum
from uuid import uuid4

from pydantic import BaseModel, Field


class TestStatus(str, Enum):
    """Test execution status."""

    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    SKIPPED = "skipped"
    TIMEOUT = "timeout"


class TestResult(BaseModel):
    """
    Test result entity capturing the outcome of executing a test scenario.

    Attributes:
        id: Unique identifier
        scenario_id: Reference to TestScenario.id
        workflow_execution_id: Reference to WorkflowExecution.id
        execution_timestamp: When the test was executed
        status: Test execution status
        duration_ms: Execution time in milliseconds
        error_message: Error message if test failed
        detected_issues: List of Issue.id detected by this test
        performance_metrics: Response time, bundle size, etc.
        captured_artifacts: Screenshots, traces, logs
    """

    id: str = Field(default_factory=lambda: str(uuid4()))
    scenario_id: str = Field(..., description="Reference to TestScenario.id")
    workflow_execution_id: str = Field(
        ..., description="Reference to WorkflowExecution.id"
    )
    execution_timestamp: datetime = Field(default_factory=datetime.utcnow)
    status: TestStatus
    duration_ms: int = Field(..., ge=0, description="Execution time in milliseconds")
    error_message: str | None = Field(None, max_length=2000)
    detected_issues: list[str] = Field(
        default_factory=list, description="List of Issue.id detected by this test"
    )
    performance_metrics: dict = Field(
        default_factory=dict, description="Response time, bundle size, etc."
    )
    captured_artifacts: dict = Field(
        default_factory=dict, description="Screenshots, traces, logs"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "id": "tr-001",
                "scenario_id": "homepage-load",
                "workflow_execution_id": "wf-20251217-143022",
                "execution_timestamp": "2025-12-17T14:30:25Z",
                "status": "passed",
                "duration_ms": 1834,
                "error_message": None,
                "detected_issues": [],
                "performance_metrics": {
                    "page_load_ms": 1834,
                    "bundle_size_kb": 285,
                    "num_requests": 12,
                },
                "captured_artifacts": {
                    "screenshot": "artifacts/screenshots/wf-20251217-143022/homepage-001.png",
                    "trace": "artifacts/traces/wf-20251217-143022/test-001-trace.zip",
                },
            }
        }
