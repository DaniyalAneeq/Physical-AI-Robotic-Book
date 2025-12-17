"""Pydantic schemas for onboarding."""

from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field, field_validator

from auth_backend.models.onboarding_profile import (
    AREAS_OF_INTEREST,
    EXPERIENCE_LEVELS,
    TOPICS_OF_INTEREST,
    USER_TYPES,
)


class OnboardingRequest(BaseModel):
    """
    Request schema for completing onboarding.

    Validates user input for the onboarding flow.

    Attributes:
        user_type: User category (must be one of predefined types)
        area_of_interest: Primary area of interest (max 100 characters)
        experience_level: Skill level (must be one of predefined levels)
        topics_of_interest: Optional list of selected topics (max 10 items)
    """

    user_type: str = Field(
        ...,
        description="User category",
        examples=["Student", "Researcher", "Teacher", "Engineer", "Other"],
    )
    area_of_interest: str = Field(
        ...,
        max_length=100,
        description="Primary area of interest",
        examples=["Robotics & Automation", "Artificial Intelligence & Machine Learning"],
    )
    experience_level: str = Field(
        ...,
        description="Skill level",
        examples=["Beginner", "Intermediate", "Advanced"],
    )
    topics_of_interest: Optional[list[str]] = Field(
        default=None,
        max_length=10,
        description="Optional list of selected topics",
        examples=[["ROS2", "NVIDIA Isaac", "Reinforcement Learning"]],
    )

    @field_validator("user_type")
    @classmethod
    def validate_user_type(cls, v: str) -> str:
        """Validate user_type is one of predefined values."""
        if v not in USER_TYPES:
            raise ValueError(
                f"user_type must be one of: {', '.join(USER_TYPES)}"
            )
        return v

    @field_validator("experience_level")
    @classmethod
    def validate_experience_level(cls, v: str) -> str:
        """Validate experience_level is one of predefined values."""
        if v not in EXPERIENCE_LEVELS:
            raise ValueError(
                f"experience_level must be one of: {', '.join(EXPERIENCE_LEVELS)}"
            )
        return v

    @field_validator("topics_of_interest")
    @classmethod
    def validate_topics(cls, v: Optional[list[str]]) -> Optional[list[str]]:
        """Validate topics_of_interest if provided."""
        if v is None:
            return v

        if len(v) > 10:
            raise ValueError("topics_of_interest cannot have more than 10 items")

        # Validate each topic is from predefined list
        invalid_topics = [topic for topic in v if topic not in TOPICS_OF_INTEREST]
        if invalid_topics:
            raise ValueError(
                f"Invalid topics: {', '.join(invalid_topics)}. "
                f"Valid topics: {', '.join(TOPICS_OF_INTEREST)}"
            )

        return v


class OnboardingStatusResponse(BaseModel):
    """
    Response schema for onboarding status check.

    Attributes:
        completed: Whether user has completed onboarding
        user_id: User's unique identifier
    """

    completed: bool = Field(..., description="Whether onboarding is completed")
    user_id: UUID = Field(..., description="User's unique identifier")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "completed": False,
                    "user_id": "a1b2c3d4-1234-5678-9012-abcdef123456",
                }
            ]
        }
    }


class OnboardingProfileResponse(BaseModel):
    """
    Response schema for onboarding profile.

    Returns user's onboarding profile data.

    Attributes:
        id: Profile unique identifier
        user_id: User's unique identifier
        user_type: User category
        area_of_interest: Primary area of interest
        experience_level: Skill level
        topics_of_interest: Selected topics (if any)
        completed_at: When onboarding was completed
    """

    id: UUID
    user_id: UUID
    user_type: str
    area_of_interest: str
    experience_level: str
    topics_of_interest: Optional[list[str]] = None
    completed_at: datetime

    model_config = {
        "from_attributes": True,
        "json_schema_extra": {
            "examples": [
                {
                    "id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
                    "user_id": "a1b2c3d4-1234-5678-9012-abcdef123456",
                    "user_type": "Researcher",
                    "area_of_interest": "Robotics & Automation",
                    "experience_level": "Intermediate",
                    "topics_of_interest": ["ROS2", "NVIDIA Isaac", "Reinforcement Learning"],
                    "completed_at": "2025-12-16T10:30:00Z",
                }
            ]
        },
    }


class OnboardingOptionsResponse(BaseModel):
    """
    Response schema for onboarding form options.

    Provides all available options for the onboarding form fields.

    Attributes:
        user_types: Available user type options
        areas_of_interest: Available area of interest options
        experience_levels: Available experience level options
        topics_of_interest: Available topic options
    """

    user_types: list[str] = Field(..., description="Available user type options")
    areas_of_interest: list[str] = Field(
        ..., description="Available area of interest options"
    )
    experience_levels: list[str] = Field(
        ..., description="Available experience level options"
    )
    topics_of_interest: list[str] = Field(..., description="Available topic options")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "user_types": ["Student", "Researcher", "Teacher", "Engineer", "Other"],
                    "areas_of_interest": [
                        "Robotics & Automation",
                        "Artificial Intelligence & Machine Learning",
                        "Computer Vision",
                        "Natural Language Processing",
                        "Hardware & Embedded Systems",
                        "Simulation & Virtual Environments",
                        "Other",
                    ],
                    "experience_levels": ["Beginner", "Intermediate", "Advanced"],
                    "topics_of_interest": [
                        "ROS2",
                        "NVIDIA Isaac",
                        "Voice Control",
                        "LLM Integration",
                        "Reinforcement Learning",
                        "Computer Vision",
                        "Path Planning",
                        "Manipulation",
                    ],
                }
            ]
        }
    }
