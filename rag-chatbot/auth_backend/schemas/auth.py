"""Authentication request/response schemas."""

from typing import Optional

from pydantic import BaseModel, EmailStr, Field

from auth_backend.schemas.user import UserResponse
from auth_backend.schemas.session import SessionResponse


class RegisterRequest(BaseModel):
    """Schema for user registration request."""

    email: EmailStr = Field(..., description="User's email address")
    name: str = Field(..., min_length=1, max_length=255, description="User's display name")
    password: str = Field(
        ...,
        min_length=8,
        max_length=128,
        description="User's password (min 8 characters)",
    )


class LoginRequest(BaseModel):
    """Schema for user login request."""

    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., description="User's password")


class AuthResponse(BaseModel):
    """Schema for authentication response (register/login)."""

    user: UserResponse = Field(..., description="User information")
    session: SessionResponse = Field(..., description="Session information")
    message: str = Field(..., description="Success message")
    onboarding_required: bool = Field(
        ...,
        description="Whether user needs to complete onboarding flow"
    )


class OAuthCallbackResponse(BaseModel):
    """Schema for OAuth callback response."""

    user: UserResponse = Field(..., description="User information")
    session: SessionResponse = Field(..., description="Session information")
    is_new_user: bool = Field(..., description="Whether this is a newly created user")
    message: str = Field(..., description="Success message")
    onboarding_required: bool = Field(
        ...,
        description="Whether user needs to complete onboarding flow"
    )


class ErrorResponse(BaseModel):
    """Schema for error responses."""

    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Error message")
    details: Optional[dict] = Field(None, description="Additional error details")
