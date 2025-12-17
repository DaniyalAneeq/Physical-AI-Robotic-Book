"""User Pydantic schemas."""

import uuid
from datetime import datetime
from typing import Optional

from pydantic import BaseModel, EmailStr, Field


class UserBase(BaseModel):
    """Base user schema with common attributes."""

    email: EmailStr = Field(..., description="User's email address")
    name: str = Field(..., min_length=1, max_length=255, description="User's display name")


class UserCreate(UserBase):
    """Schema for creating a new user."""

    password: str = Field(
        ...,
        min_length=8,
        max_length=128,
        description="User's password (min 8 characters)",
    )


class UserResponse(UserBase):
    """Schema for user response (excludes password)."""

    id: uuid.UUID = Field(..., description="Unique user identifier")
    email_verified_at: Optional[datetime] = Field(
        None, description="Timestamp when email was verified"
    )
    created_at: datetime = Field(..., description="Account creation timestamp")
    updated_at: datetime = Field(..., description="Last account update timestamp")

    class Config:
        """Pydantic config."""

        from_attributes = True
