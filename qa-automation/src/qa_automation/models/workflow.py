"""Workflow execution entity model."""

from datetime import datetime
from enum import Enum

from pydantic import BaseModel, Field


class WorkflowStatus(str, Enum):
    """Overall workflow status."""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class PhaseResult(BaseModel):
    """
    Result of a workflow phase.

    Attributes:
        phase_name: Name of the phase
        duration_seconds: Time taken to complete phase
        status: Phase completion status
        items_processed: Total items processed in phase
        items_successful: Successfully processed items
        items_failed: Failed items
    """

    phase_name: str
    duration_seconds: int = Field(..., ge=0)
    status: str
    items_processed: int = Field(..., ge=0)
    items_successful: int = Field(..., ge=0)
    items_failed: int = Field(..., ge=0)


class WorkflowSummary(BaseModel):
    """
    Summary statistics for workflow.

    Attributes:
        total_issues_detected: Total number of issues found
        critical_issues: Count of critical issues
        high_issues: Count of high severity issues
        medium_issues: Count of medium severity issues
        low_issues: Count of low severity issues
        auto_fixed: Count of automatically fixed issues
        manual_required: Count of issues requiring manual intervention
        regressions_detected: Count of new issues introduced by fixes
    """

    total_issues_detected: int = Field(..., ge=0)
    critical_issues: int = Field(..., ge=0)
    high_issues: int = Field(..., ge=0)
    medium_issues: int = Field(..., ge=0)
    low_issues: int = Field(..., ge=0)
    auto_fixed: int = Field(..., ge=0)
    manual_required: int = Field(..., ge=0)
    regressions_detected: int = Field(..., ge=0)


class WorkflowExecution(BaseModel):
    """
    Workflow execution entity tracking a complete run of the QA workflow.

    Attributes:
        id: Unique workflow identifier
        start_time: When workflow started
        end_time: When workflow ended
        duration_seconds: Total duration
        status: Overall workflow status
        config: Configuration used for this run
        deployment_ready: Whether app is ready for deployment
        phase_results: Results for each phase
        summary: Summary statistics
        test_results: List of TestResult.id
        detected_issues: List of Issue.id
        resolution_actions: List of ResolutionAction.id
        artifacts: Paths to screenshots, traces, logs, reports
        original_workflow_id: If this is a re-test, ID of original workflow
        retest_iteration: Re-test iteration number (0 = initial run)
    """

    id: str = Field(
        default_factory=lambda: f"wf-{datetime.utcnow().strftime('%Y%m%d-%H%M%S')}"
    )
    start_time: datetime = Field(default_factory=datetime.utcnow)
    end_time: datetime | None = None
    duration_seconds: int | None = Field(None, ge=0)
    status: WorkflowStatus = WorkflowStatus.PENDING
    config: dict = Field(..., description="Configuration used for this run")
    deployment_ready: bool | None = Field(
        None, description="Whether app is ready for deployment"
    )

    # Phase results
    phase_results: list[PhaseResult] = Field(default_factory=list)

    # Summary
    summary: WorkflowSummary | None = None

    # References
    test_results: list[str] = Field(
        default_factory=list, description="List of TestResult.id"
    )
    detected_issues: list[str] = Field(
        default_factory=list, description="List of Issue.id"
    )
    resolution_actions: list[str] = Field(
        default_factory=list, description="List of ResolutionAction.id"
    )

    # Artifacts
    artifacts: dict = Field(
        default_factory=dict,
        description="Paths to screenshots, traces, logs, reports",
    )

    # Re-test tracking
    original_workflow_id: str | None = Field(
        None, description="If this is a re-test, ID of original workflow"
    )
    retest_iteration: int = Field(0, description="Re-test iteration number (0 = initial run)")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "wf-20251217-143022",
                "start_time": "2025-12-17T14:30:22Z",
                "end_time": "2025-12-17T14:58:15Z",
                "duration_seconds": 1673,
                "status": "completed",
                "deployment_ready": True,
                "config": {
                    "frontend_url": "http://localhost:3000",
                    "backend_url": "https://e-book-physical-ai-humanoid-robotics.onrender.com",
                    "max_retest_iterations": 3,
                },
                "phase_results": [
                    {
                        "phase_name": "detection",
                        "duration_seconds": 832,
                        "status": "completed",
                        "items_processed": 150,
                        "items_successful": 148,
                        "items_failed": 2,
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
                    "regressions_detected": 1,
                },
                "test_results": ["tr-001", "tr-002"],
                "detected_issues": ["iss-001", "iss-002"],
                "resolution_actions": ["ra-001"],
                "artifacts": {
                    "screenshots": "artifacts/screenshots/wf-20251217-143022/",
                    "traces": "artifacts/traces/wf-20251217-143022/",
                    "logs": "artifacts/logs/workflow-wf-20251217-143022.log",
                    "report_json": "reports/workflow-wf-20251217-143022.json",
                    "report_html": "reports/dashboard-wf-20251217-143022.html",
                },
                "original_workflow_id": None,
                "retest_iteration": 0,
            }
        }
