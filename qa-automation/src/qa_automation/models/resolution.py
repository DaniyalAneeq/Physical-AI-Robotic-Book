"""Resolution action entity model."""

from datetime import datetime
from enum import Enum
from uuid import uuid4

from pydantic import BaseModel, Field


class ActionType(str, Enum):
    """Type of resolution action."""

    AUTO_FIX = "auto_fix"
    AUTO_FIX_FAILED = "auto_fix_failed"
    MANUAL_GUIDANCE = "manual_guidance"


class VerificationStatus(str, Enum):
    """Result of fix verification."""

    PASSED = "passed"
    FAILED = "failed"
    NOT_VERIFIED = "not_verified"


class ResolutionOutcome(str, Enum):
    """Final outcome of resolution."""

    FIXED = "fixed"
    MANUAL_REQUIRED = "manual_required"
    ROLLBACK = "rollback"


class ResolutionAction(BaseModel):
    """
    Resolution action entity representing an attempted fix for an issue.

    Attributes:
        id: Unique identifier
        issue_id: Reference to Issue.id
        workflow_execution_id: Reference to WorkflowExecution.id
        action_timestamp: When the action was taken
        action_type: Type of resolution action
        applied_changes: Git diff or description of changes
        guidance: Manual resolution guidance
        verification_status: Result of fix verification
        outcome: Final outcome of resolution
    """

    id: str = Field(default_factory=lambda: str(uuid4()))
    issue_id: str = Field(..., description="Reference to Issue.id")
    workflow_execution_id: str = Field(
        ..., description="Reference to WorkflowExecution.id"
    )
    action_timestamp: datetime = Field(default_factory=datetime.utcnow)
    action_type: ActionType
    applied_changes: str | None = Field(
        None, max_length=10000, description="Git diff or description of changes"
    )
    guidance: str | None = Field(
        None, max_length=5000, description="Manual resolution guidance"
    )
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
                "applied_changes": '--- a/src/components/ProfileImage.tsx\n+++ b/src/components/ProfileImage.tsx\n@@ -10,7 +10,7 @@\n-  <img src={avatarUrl} className="profile-avatar" />\n+  <img src={avatarUrl} alt="User profile avatar" className="profile-avatar" />',
                "guidance": None,
                "verification_status": "passed",
                "outcome": "fixed",
            }
        }
