"""
Content API endpoints for retrieving localized book module content.

Provides endpoints for fetching module content in the user's preferred language.
Falls back to English if translations are unavailable.
"""

import os
import sys
from pathlib import Path
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, Header, status
from pydantic import BaseModel, Field
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from sqlalchemy.ext.asyncio import AsyncSession

# Add auth_backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "auth_backend"))

from app.utils.locale import get_user_locale
from auth_backend.models.user import User
from auth_backend.api.deps import get_current_user
from auth_backend.database import get_db

router = APIRouter(prefix="/api/content", tags=["content"])


class LocalizedContent(BaseModel):
    """Response model for localized content."""

    module_id: str = Field(..., description="Unique identifier for the module")
    locale: str = Field(..., description="Language code of returned content (en or ur)")
    content: str = Field(..., description="Module content in requested language")
    title: str = Field(..., description="Module title in requested language")
    translation_status: str = Field(
        ...,
        description="Translation status: complete, partial, or unavailable"
    )
    fallback_message: Optional[str] = Field(
        None,
        description="Message shown when translation is incomplete"
    )


@router.get("/{module_id}", response_model=LocalizedContent)
async def get_module_content(
    module_id: str,
    current_user: User = Depends(get_current_user),
    accept_language: Optional[str] = Header(None, alias="Accept-Language")
) -> LocalizedContent:
    """
    Retrieve module content in the user's preferred language.

    Args:
        module_id: The module identifier (e.g., "module-1-chapter-1")
        current_user: Authenticated user (from JWT token)
        accept_language: Accept-Language header (optional, falls back to user preference)

    Returns:
        LocalizedContent with content in requested language or fallback to English

    Raises:
        HTTPException: 404 if module not found, 500 for Qdrant errors
    """
    # Determine user's preferred locale
    user_locale = get_user_locale(
        accept_language_header=accept_language,
        user_preferred_locale=current_user.preferred_locale
    )

    # Initialize Qdrant client
    # TODO: Move to dependency injection for better testability
    try:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL", "http://localhost:6333"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to connect to Qdrant: {str(e)}"
        )

    # Query Qdrant for module by module_id
    try:
        search_results = qdrant_client.scroll(
            collection_name="textbook_chunks",
            scroll_filter=Filter(
                must=[
                    FieldCondition(
                        key="module_id",
                        match=MatchValue(value=module_id)
                    )
                ]
            ),
            limit=1,
            with_payload=True,
            with_vectors=False
        )

        points, _ = search_results

        if not points:
            raise HTTPException(
                status_code=404,
                detail=f"Module '{module_id}' not found"
            )

        module_data = points[0].payload

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error querying Qdrant: {str(e)}"
        )

    # Select content based on locale
    if user_locale == "ur":
        content = module_data.get("content_ur", "")
        title = module_data.get("title_ur", "")
        translation_status = module_data.get("translation_status", "unavailable")

        # Fallback to English if Urdu translation is incomplete
        if translation_status != "complete" or not content:
            content = module_data.get("content_en", "")
            title = module_data.get("title_en", "")
            fallback_message = (
                "This content is not yet available in Urdu. "
                "Showing English version. Translation coming soon!"
            )
            actual_locale = "en"
        else:
            fallback_message = None
            actual_locale = "ur"
    else:
        # English locale
        content = module_data.get("content_en", "")
        title = module_data.get("title_en", "")
        translation_status = "complete"
        fallback_message = None
        actual_locale = "en"

    return LocalizedContent(
        module_id=module_id,
        locale=actual_locale,
        content=content,
        title=title,
        translation_status=translation_status,
        fallback_message=fallback_message
    )
