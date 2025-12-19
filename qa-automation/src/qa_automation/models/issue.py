"""Issue entity model."""

from datetime import datetime
from enum import Enum
from uuid import uuid4

from pydantic import BaseModel, Field


class IssueType(str, Enum):
    """Type of issue detected."""

    FRONTEND = "frontend"
    BACKEND = "backend"
    PERFORMANCE = "performance"


class SeverityLevel(str, Enum):
    """Severity classification."""

    CRITICAL = "critical"  # Blocks deployment
    HIGH = "high"  # Significant impact
    MEDIUM = "medium"  # Moderate impact
    LOW = "low"  # Minor impact


class ResolutionStatus(str, Enum):
    """Current resolution state."""

    PENDING = "pending"
    AUTO_FIXED = "auto_fixed"
    MANUAL_REQUIRED = "manual_required"
    VERIFIED = "verified"
    WONT_FIX = "wont_fix"


class Issue(BaseModel):
    """
    Issue entity representing a detected problem in the application.

    Attributes:
        id: Unique identifier
        type: Type of issue (frontend/backend/performance)
        severity: Severity level (assigned in analysis phase)
        detection_timestamp: When the issue was detected
        location: URL/endpoint/file path where issue was detected
        description: Human-readable description
        captured_context: Screenshots, request/response data, logs
        root_cause: Identified root cause (assigned in analysis)
        related_issues: List of related issue IDs
        resolution_status: Current resolution state
        resolution_action_id: ID of resolution action taken
    """

    id: str = Field(default_factory=lambda: str(uuid4()))
    type: IssueType
    severity: SeverityLevel | None = None
    detection_timestamp: datetime = Field(default_factory=datetime.utcnow)
    location: str = Field(
        ..., description="URL/endpoint/file path where issue was detected"
    )
    description: str = Field(..., min_length=10, max_length=1000)
    captured_context: dict = Field(
        default_factory=dict,
        description="Screenshots, request/response data, logs",
    )
    root_cause: str | None = Field(
        None, description="Identified root cause (assigned in analysis)"
    )
    related_issues: list[str] = Field(
        default_factory=list, description="List of related issue IDs"
    )
    resolution_status: ResolutionStatus = ResolutionStatus.PENDING
    resolution_action_id: str | None = Field(
        None, description="ID of resolution action taken"
    )

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
                    "html": "<img src='/avatar.jpg' class='profile-avatar'>",
                },
                "root_cause": "Direct issue - no upstream dependency detected",
                "related_issues": ["iss-002", "iss-003"],
                "resolution_status": "pending",
                "resolution_action_id": None,
            }
        }
