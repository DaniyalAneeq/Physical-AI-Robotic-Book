"""
User Preferences API Endpoints

Handles user preference management including locale/language settings.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
import sys
from pathlib import Path

# Add auth_backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "auth_backend"))

from auth_backend.models.user import User
from auth_backend.api.deps import get_current_user
from auth_backend.database import get_db

router = APIRouter(prefix="/api/user", tags=["user"])


# ============================================
# Pydantic Schemas
# ============================================


class UserPreferencesResponse(BaseModel):
    """Response model for user preferences"""
    user_id: str
    email: str
    preferred_locale: str

    class Config:
        from_attributes = True


class UpdatePreferencesRequest(BaseModel):
    """Request model for updating user preferences"""
    preferred_locale: Optional[str] = Field(
        None,
        description="User's preferred locale (en or ur)",
        pattern="^(en|ur)$",
    )


# ============================================
# API Endpoints
# ============================================


@router.get("/preferences", response_model=UserPreferencesResponse)
async def get_user_preferences(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """
    Get current user's preferences.

    Returns:
        UserPreferencesResponse: User preferences including preferred locale

    Raises:
        401: If user not authenticated
    """
    return UserPreferencesResponse(
        user_id=str(user.id),
        email=user.email,
        preferred_locale=user.preferred_locale,
    )


@router.put("/preferences", response_model=UserPreferencesResponse)
async def update_user_preferences(
    data: UpdatePreferencesRequest,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """
    Update user preferences.

    Args:
        data: Updated preference values

    Returns:
        UserPreferencesResponse: Updated user preferences

    Raises:
        400: If preferred_locale is invalid
        401: If user not authenticated
    """
    # Validate locale if provided
    if data.preferred_locale:
        if data.preferred_locale not in ['en', 'ur']:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid locale. Must be 'en' or 'ur'.",
            )

        # Update user's preferred locale
        user.preferred_locale = data.preferred_locale
        await db.commit()
        await db.refresh(user)

    return UserPreferencesResponse(
        user_id=str(user.id),
        email=user.email,
        preferred_locale=user.preferred_locale,
    )


# ============================================
# Additional Helper Endpoints
# ============================================


@router.get("/me", response_model=UserPreferencesResponse)
async def get_current_user_info(
    user: User = Depends(get_current_user),
):
    """
    Get current authenticated user info (alias for /preferences).

    Returns:
        UserPreferencesResponse: Current user information
    """
    return UserPreferencesResponse(
        user_id=str(user.id),
        email=user.email,
        preferred_locale=user.preferred_locale,
    )
