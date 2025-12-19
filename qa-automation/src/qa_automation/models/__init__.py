"""
Data models for QA Automation workflow.

All models use Pydantic for validation and serialization.
"""

from qa_automation.models.issue import Issue, IssueType, SeverityLevel, ResolutionStatus
from qa_automation.models.test_scenario import (
    TestScenario,
    TestType,
    AuthMethod,
    TestStep,
    Assertion,
)
from qa_automation.models.test_result import TestResult, TestStatus
from qa_automation.models.resolution import (
    ResolutionAction,
    ActionType,
    VerificationStatus,
    ResolutionOutcome,
)
from qa_automation.models.workflow import (
    WorkflowExecution,
    WorkflowStatus,
    PhaseResult,
    WorkflowSummary,
)

__all__ = [
    # Issue models
    "Issue",
    "IssueType",
    "SeverityLevel",
    "ResolutionStatus",
    # Test scenario models
    "TestScenario",
    "TestType",
    "AuthMethod",
    "TestStep",
    "Assertion",
    # Test result models
    "TestResult",
    "TestStatus",
    # Resolution models
    "ResolutionAction",
    "ActionType",
    "VerificationStatus",
    "ResolutionOutcome",
    # Workflow models
    "WorkflowExecution",
    "WorkflowStatus",
    "PhaseResult",
    "WorkflowSummary",
]
