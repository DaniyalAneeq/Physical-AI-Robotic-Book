"""Session Pydantic schemas."""

import uuid
from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class SessionResponse(BaseModel):
    """Schema for session response."""

    id: uuid.UUID = Field(..., description="Unique session identifier")
    user_id: uuid.UUID = Field(..., description="User ID associated with session")
    last_used_at: datetime = Field(..., description="Last session use timestamp")
    expires_at: datetime = Field(..., description="Session expiration timestamp")
    ip_address: Optional[str] = Field(None, description="IP address of session creation")
    user_agent: Optional[str] = Field(None, description="User agent string")
    created_at: datetime = Field(..., description="Session creation timestamp")

    class Config:
        """Pydantic config."""

        from_attributes = True
