"""OnboardingProfile SQLAlchemy model."""

import uuid
from datetime import datetime, timezone
from typing import Optional

from sqlalchemy import CheckConstraint, DateTime, ForeignKey, String
from sqlalchemy.dialects.postgresql import JSONB, UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from auth_backend.models.base import Base


class OnboardingProfile(Base):
    """
    Onboarding profile model.

    Stores user profile data collected during the mandatory onboarding flow.

    Attributes:
        id: Unique profile identifier
        user_id: Foreign key to users table (one profile per user)
        user_type: User category (Student, Researcher, Teacher, Engineer, Other)
        area_of_interest: Primary area of interest
        experience_level: Skill level (Beginner, Intermediate, Advanced)
        topics_of_interest: Array of selected topics (optional)
        completed_at: Onboarding completion timestamp
        created_at: Record creation timestamp
        updated_at: Last update timestamp
    """

    __tablename__ = "onboarding_profiles"

    # Primary key
    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
    )

    # Foreign key to users (one-to-one relationship)
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        unique=True,
        nullable=False,
    )

    # Profile fields
    user_type: Mapped[str] = mapped_column(String(50), nullable=False)
    area_of_interest: Mapped[str] = mapped_column(String(100), nullable=False)
    experience_level: Mapped[str] = mapped_column(String(20), nullable=False)
    topics_of_interest: Mapped[Optional[list[str]]] = mapped_column(
        JSONB, nullable=True
    )

    # Timestamps
    completed_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        default=lambda: datetime.now(timezone.utc),
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        default=lambda: datetime.now(timezone.utc),
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        default=lambda: datetime.now(timezone.utc),
        onupdate=lambda: datetime.now(timezone.utc),
    )

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="onboarding_profile")

    # Table constraints
    __table_args__ = (
        CheckConstraint(
            "user_type IN ('Student', 'Teacher', 'Researcher', 'Engineer')",
            name="user_type_check",
        ),
        CheckConstraint(
            "experience_level IN ('Beginner', 'Intermediate', 'Advanced')",
            name="experience_level_check",
        ),
    )

    def __repr__(self) -> str:
        """String representation of OnboardingProfile."""
        return (
            f"<OnboardingProfile(id={self.id}, user_id={self.user_id}, "
            f"user_type={self.user_type}, experience={self.experience_level})>"
        )


# Predefined values for validation (application-level)
USER_TYPES = ["Student", "Teacher", "Researcher", "Engineer"]

AREAS_OF_INTEREST = [
    "NVIDIA Isaac",
    "LLM Integration",
    "Reinforcement Learning",
    "Robotics",
]

EXPERIENCE_LEVELS = ["Beginner", "Intermediate", "Advanced"]

# Topics are same as areas of interest (per requirements)
TOPICS_OF_INTEREST = AREAS_OF_INTEREST
