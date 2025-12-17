"""Onboarding routes for user profile collection."""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from auth_backend.api.deps import get_current_user
from auth_backend.database import get_db
from auth_backend.models.user import User
from auth_backend.schemas.onboarding import (
    OnboardingOptionsResponse,
    OnboardingProfileResponse,
    OnboardingRequest,
)
from auth_backend.services.onboarding import onboarding_service

router = APIRouter(tags=["Onboarding"])


@router.get("/options", response_model=OnboardingOptionsResponse)
async def get_onboarding_options() -> OnboardingOptionsResponse:
    """
    Get predefined onboarding options.

    Returns all available options for user type, areas of interest,
    experience levels, and topics. Used by frontend for dropdowns.

    **Authentication Required:** No (public endpoint)

    **Response:**
    - user_types: List of valid user types
    - areas_of_interest: List of valid areas
    - experience_levels: List of valid experience levels
    - topics_of_interest: List of valid topics

    Example:
        ```
        GET /onboarding/options
        -> {
            "user_types": ["student", "educator", "researcher", "hobbyist", "professional"],
            "areas_of_interest": ["robotics", "ai_ml", "iot", "automation", "research"],
            "experience_levels": ["beginner", "intermediate", "advanced"],
            "topics_of_interest": ["ros2", "computer_vision", "nlp", ...]
        }
        ```
    """
    options = onboarding_service.get_predefined_options()
    return OnboardingOptionsResponse(**options)


@router.post("/complete", response_model=OnboardingProfileResponse, status_code=status.HTTP_201_CREATED)
async def complete_onboarding(
    profile_data: OnboardingRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
) -> OnboardingProfileResponse:
    """
    Complete onboarding by submitting profile.

    Creates onboarding profile and marks user as onboarding_completed.
    This endpoint should be called after registration or login if
    onboarding_required is true.

    **Authentication Required:** Yes (via session cookie)

    **Request Body:**
    - user_type: Type of user (student, educator, researcher, etc.)
    - area_of_interest: Primary area of interest
    - experience_level: Experience level (beginner, intermediate, advanced)
    - topics_of_interest: Optional list of specific topics (max 10)

    **Response:**
    - Created onboarding profile

    **Errors:**
    - 400 Bad Request: Invalid input or user already has profile
    - 401 Unauthorized: Not authenticated

    Example:
        ```json
        POST /onboarding/complete
        Cookie: session_token=abc123xyz
        {
          "user_type": "student",
          "area_of_interest": "robotics",
          "experience_level": "beginner",
          "topics_of_interest": ["ros2", "simulation"]
        }
        ```
    """
    try:
        profile = await onboarding_service.create_profile(db, user.id, profile_data)
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )

    return OnboardingProfileResponse.model_validate(profile)


@router.get("/profile", response_model=OnboardingProfileResponse)
async def get_onboarding_profile(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
) -> OnboardingProfileResponse:
    """
    Get current user's onboarding profile.

    Returns the user's onboarding profile if it exists.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - Onboarding profile information

    **Errors:**
    - 401 Unauthorized: Not authenticated
    - 404 Not Found: Profile not found (user hasn't completed onboarding)

    Example:
        ```
        GET /onboarding/profile
        Cookie: session_token=abc123xyz
        -> {
            "id": "...",
            "user_id": "...",
            "user_type": "student",
            "area_of_interest": "robotics",
            "experience_level": "beginner",
            "topics_of_interest": ["ros2", "simulation"],
            "created_at": "2025-01-15T10:30:00Z",
            "updated_at": "2025-01-15T10:30:00Z"
        }
        ```
    """
    profile = await onboarding_service.get_profile(db, user.id)

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Onboarding profile not found. Please complete onboarding first.",
        )

    return OnboardingProfileResponse.model_validate(profile)


@router.put("/profile", response_model=OnboardingProfileResponse)
async def update_onboarding_profile(
    profile_data: OnboardingRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
) -> OnboardingProfileResponse:
    """
    Update current user's onboarding profile.

    Allows users to update their profile after initial creation.
    All fields must be provided (full update, not partial).

    **Authentication Required:** Yes (via session cookie)

    **Request Body:**
    - user_type: Updated user type
    - area_of_interest: Updated area of interest
    - experience_level: Updated experience level
    - topics_of_interest: Updated topics list (optional)

    **Response:**
    - Updated onboarding profile

    **Errors:**
    - 400 Bad Request: Invalid input
    - 401 Unauthorized: Not authenticated
    - 404 Not Found: Profile not found

    Example:
        ```json
        PUT /onboarding/profile
        Cookie: session_token=abc123xyz
        {
          "user_type": "researcher",
          "area_of_interest": "ai_ml",
          "experience_level": "intermediate",
          "topics_of_interest": ["computer_vision", "nlp"]
        }
        ```
    """
    try:
        profile = await onboarding_service.update_profile(db, user.id, profile_data)
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e),
        )

    return OnboardingProfileResponse.model_validate(profile)


@router.get("/status")
async def get_onboarding_status(
    user: User = Depends(get_current_user),
) -> dict:
    """
    Check if user has completed onboarding.

    Simple status check endpoint to determine if user needs
    to complete onboarding flow.

    **Authentication Required:** Yes (via session cookie)

    **Response:**
    - onboarding_completed: Boolean indicating completion status
    - user_id: User's UUID

    **Errors:**
    - 401 Unauthorized: Not authenticated

    Example:
        ```
        GET /onboarding/status
        Cookie: session_token=abc123xyz
        -> {
            "user_id": "...",
            "onboarding_completed": true
        }
        ```
    """
    return {
        "user_id": str(user.id),
        "onboarding_completed": user.onboarding_completed,
    }
