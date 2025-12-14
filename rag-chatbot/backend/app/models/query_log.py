"""QueryLog model for audit logging."""

import uuid
from datetime import datetime

from sqlalchemy import DateTime, Index, Integer, Text, func
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column

from app.models.database import Base


class QueryLog(Base):
    """Audit log for debugging and analytics (anonymized)."""

    __tablename__ = "query_logs"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
    )
    query_text: Mapped[str] = mapped_column(
        Text,
        nullable=False,
        comment="User query (no PII)",
    )
    response_summary: Mapped[str | None] = mapped_column(
        Text,
        nullable=True,
        comment="First 200 chars of response",
    )
    latency_ms: Mapped[int | None] = mapped_column(
        Integer,
        nullable=True,
        comment="Query-to-response time in ms",
    )
    chunks_retrieved: Mapped[int | None] = mapped_column(
        Integer,
        nullable=True,
        comment="Number of chunks from Qdrant",
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now(),
    )

    __table_args__ = (
        Index("idx_querylog_created_at", "created_at"),
    )

    def __repr__(self) -> str:
        return f"<QueryLog(id={self.id}, latency_ms={self.latency_ms})>"
