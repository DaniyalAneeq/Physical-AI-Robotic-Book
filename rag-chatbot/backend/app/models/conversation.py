"""Conversation model for chat sessions."""

import uuid
from datetime import datetime

from sqlalchemy import DateTime, Index, String, func
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from app.models.database import Base


class Conversation(Base):
    """Represents a chat session identified by browser-generated UUID."""

    __tablename__ = "conversations"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
    )
    session_id: Mapped[str] = mapped_column(
        String(255),
        nullable=False,
        index=True,
        comment="Browser localStorage UUID",
    )
    title: Mapped[str | None] = mapped_column(
        String(255),
        nullable=True,
        comment="Auto-generated or user-set title",
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now(),
    )
    updated_at: Mapped[datetime | None] = mapped_column(
        DateTime(timezone=True),
        nullable=True,
        onupdate=func.now(),
    )

    # Relationship to messages
    messages: Mapped[list["Message"]] = relationship(
        "Message",
        back_populates="conversation",
        cascade="all, delete-orphan",
        order_by="Message.created_at",
    )

    __table_args__ = (
        Index("idx_conversation_created_at", "created_at"),
    )

    def __repr__(self) -> str:
        return f"<Conversation(id={self.id}, title={self.title})>"


# Import Message for type hints (avoid circular import at runtime)
from app.models.message import Message  # noqa: E402, F401
