"""Onboarding service for user profile collection."""

from typing import Optional
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.models.onboarding_profile import (
    AREAS_OF_INTEREST,
    EXPERIENCE_LEVELS,
    OnboardingProfile,
    TOPICS_OF_INTEREST,
    USER_TYPES,
)
from auth_backend.models.user import User
from auth_backend.schemas.onboarding import (
    OnboardingProfileResponse,
    OnboardingRequest,
)


class OnboardingService:
    """
    Service for managing user onboarding profiles.

    Handles profile creation, validation, and retrieval.
    Follows Better Auth patterns for user lifecycle management.
    """

    # Expose predefined options as class attributes
    USER_TYPES = USER_TYPES
    AREAS_OF_INTEREST = AREAS_OF_INTEREST
    EXPERIENCE_LEVELS = EXPERIENCE_LEVELS
    TOPICS_OF_INTEREST = TOPICS_OF_INTEREST

    async def create_profile(
        self, db: AsyncSession, user_id: UUID, profile_data: OnboardingRequest
    ) -> OnboardingProfile:
        """
        Create onboarding profile for a user.

        Validates input against predefined options and creates profile record.
        Marks user as onboarding_completed after successful creation.

        Args:
            db: Database session
            user_id: User's UUID
            profile_data: Validated onboarding data

        Returns:
            OnboardingProfile: Created profile

        Raises:
            ValueError: If user already has a profile
            ValueError: If user not found

        Example:
            >>> profile_data = OnboardingRequest(
            ...     user_type="student",
            ...     area_of_interest="robotics",
            ...     experience_level="beginner",
            ...     topics_of_interest=["ros2", "simulation"]
            ... )
            >>> profile = await service.create_profile(db, user.id, profile_data)
        """
        # Check if user exists
        stmt = select(User).where(User.id == user_id)
        result = await db.execute(stmt)
        user = result.scalar_one_or_none()

        if not user:
            raise ValueError(f"User with id {user_id} not found")

        # Check if profile already exists
        stmt = select(OnboardingProfile).where(OnboardingProfile.user_id == user_id)
        result = await db.execute(stmt)
        existing_profile = result.scalar_one_or_none()

        if existing_profile:
            raise ValueError("User already has an onboarding profile")

        # Create profile
        profile = OnboardingProfile(
            user_id=user_id,
            user_type=profile_data.user_type,
            area_of_interest=profile_data.area_of_interest,
            experience_level=profile_data.experience_level,
            topics_of_interest=profile_data.topics_of_interest,
        )
        db.add(profile)

        # Mark user as onboarding completed
        user.onboarding_completed = True

        await db.commit()
        await db.refresh(profile)

        return profile

    async def get_profile(
        self, db: AsyncSession, user_id: UUID
    ) -> Optional[OnboardingProfile]:
        """
        Get onboarding profile for a user.

        Args:
            db: Database session
            user_id: User's UUID

        Returns:
            Optional[OnboardingProfile]: Profile if exists, None otherwise

        Example:
            >>> profile = await service.get_profile(db, user.id)
            >>> if profile:
            ...     print(f"User type: {profile.user_type}")
        """
        stmt = select(OnboardingProfile).where(OnboardingProfile.user_id == user_id)
        result = await db.execute(stmt)
        return result.scalar_one_or_none()

    async def update_profile(
        self,
        db: AsyncSession,
        user_id: UUID,
        profile_data: OnboardingRequest,
    ) -> OnboardingProfile:
        """
        Update existing onboarding profile.

        Args:
            db: Database session
            user_id: User's UUID
            profile_data: New profile data

        Returns:
            OnboardingProfile: Updated profile

        Raises:
            ValueError: If profile not found

        Example:
            >>> updated_data = OnboardingRequest(
            ...     user_type="researcher",
            ...     area_of_interest="ai_ml",
            ...     experience_level="intermediate",
            ...     topics_of_interest=["computer_vision", "nlp"]
            ... )
            >>> profile = await service.update_profile(db, user.id, updated_data)
        """
        stmt = select(OnboardingProfile).where(OnboardingProfile.user_id == user_id)
        result = await db.execute(stmt)
        profile = result.scalar_one_or_none()

        if not profile:
            raise ValueError("Onboarding profile not found")

        # Update fields
        profile.user_type = profile_data.user_type
        profile.area_of_interest = profile_data.area_of_interest
        profile.experience_level = profile_data.experience_level
        profile.topics_of_interest = profile_data.topics_of_interest

        await db.commit()
        await db.refresh(profile)

        return profile

    def get_predefined_options(self) -> dict[str, list[str]]:
        """
        Get all predefined onboarding options.

        Returns a dictionary with all available options for each field.
        Useful for frontend dropdowns and validation.

        Returns:
            dict: Dictionary with keys:
                - user_types: List of valid user types
                - areas_of_interest: List of valid areas
                - experience_levels: List of valid levels
                - topics_of_interest: List of valid topics

        Example:
            >>> options = service.get_predefined_options()
            >>> print(options["user_types"])
            ['student', 'educator', 'researcher', 'hobbyist', 'professional']
        """
        return {
            "user_types": list(self.USER_TYPES),
            "areas_of_interest": list(self.AREAS_OF_INTEREST),
            "experience_levels": list(self.EXPERIENCE_LEVELS),
            "topics_of_interest": list(self.TOPICS_OF_INTEREST),
        }


# Singleton instance
onboarding_service = OnboardingService()
