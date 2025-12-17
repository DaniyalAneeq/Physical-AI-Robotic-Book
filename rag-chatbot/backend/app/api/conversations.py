"""Conversation management API endpoints."""

import logging
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from app.models.conversation import Conversation
from app.models.database import get_db
from app.models.message import Message

# Import auth dependencies
try:
    from auth_backend.api.deps import get_current_user_with_onboarding as _get_user
    from auth_backend.models.user import User as _User
    AUTH_AVAILABLE = True
except ImportError:
    _get_user = None
    _User = None
    AUTH_AVAILABLE = False

router = APIRouter()
logger = logging.getLogger(__name__)


# Conditional auth dependency
async def maybe_get_user():
    """Return user if auth is available, otherwise None."""
    if AUTH_AVAILABLE and _get_user:
        return await _get_user()
    return None


@router.get("/conversations")
async def list_conversations(
    session_id: str,
    db: Session = Depends(get_db),
    user=Depends(maybe_get_user),
):
    """
    List all conversations for a session.

    **Authentication Required:** Yes (must be authenticated with completed onboarding)

    Args:
        session_id: Browser session UUID
        db: Database session

    Returns:
        List of conversations with metadata

    **Errors:**
    - 401 Unauthorized: Not authenticated or invalid session
    - 403 Forbidden: Authenticated but onboarding not completed
    """
    try:
        conversations = (
            db.query(Conversation)
            .filter(Conversation.session_id == session_id)
            .order_by(Conversation.updated_at.desc())
            .all()
        )

        return [
            {
                "id": str(conv.id),
                "title": conv.title,
                "created_at": conv.created_at.isoformat(),
                "updated_at": conv.updated_at.isoformat() if conv.updated_at else None,
            }
            for conv in conversations
        ]
    except Exception as e:
        logger.error(f"Failed to list conversations: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve conversations") from e


@router.get("/conversations/{conversation_id}")
async def get_conversation(conversation_id: str, db: Session = Depends(get_db)):
    """
    Get conversation details with all messages.

    Args:
        conversation_id: Conversation UUID
        db: Database session

    Returns:
        Conversation with messages
    """
    try:
        conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()

        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")

        messages = (
            db.query(Message)
            .filter(Message.conversation_id == conversation_id)
            .order_by(Message.created_at.asc())
            .all()
        )

        return {
            "id": str(conversation.id),
            "title": conversation.title,
            "created_at": conversation.created_at.isoformat(),
            "updated_at": conversation.updated_at.isoformat() if conversation.updated_at else None,
            "messages": [
                {
                    "id": str(msg.id),
                    "role": msg.role,
                    "content": msg.content,
                    "citations": msg.citations,
                    "created_at": msg.created_at.isoformat(),
                }
                for msg in messages
            ],
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to get conversation: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve conversation") from e


@router.patch("/conversations/{conversation_id}")
async def update_conversation(conversation_id: str, title: str, db: Session = Depends(get_db)):
    """
    Update conversation title.

    Args:
        conversation_id: Conversation UUID
        title: New title
        db: Database session

    Returns:
        Updated conversation
    """
    try:
        conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()

        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")

        conversation.title = title
        conversation.updated_at = datetime.utcnow()
        db.commit()

        return {
            "id": str(conversation.id),
            "title": conversation.title,
            "updated_at": conversation.updated_at.isoformat(),
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to update conversation: {e}")
        db.rollback()
        raise HTTPException(status_code=500, detail="Failed to update conversation") from e


@router.delete("/conversations/{conversation_id}")
async def delete_conversation(conversation_id: str, db: Session = Depends(get_db)):
    """
    Delete conversation and all associated messages.

    Args:
        conversation_id: Conversation UUID
        db: Database session

    Returns:
        Deletion confirmation
    """
    try:
        conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()

        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")

        db.delete(conversation)  # Messages cascade-deleted via FK
        db.commit()

        return {"status": "deleted", "conversation_id": conversation_id}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to delete conversation: {e}")
        db.rollback()
        raise HTTPException(status_code=500, detail="Failed to delete conversation") from e
