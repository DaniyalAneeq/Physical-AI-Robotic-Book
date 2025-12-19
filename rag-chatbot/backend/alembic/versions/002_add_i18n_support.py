"""Add i18n support - translation cache table.

Revision ID: 002
Revises: 001
Create Date: 2025-12-17

"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "002"
down_revision: Union[str, None] = "001"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Create translation_cache table for chatbot query translations."""

    # Create translation_cache table
    op.create_table(
        "translation_cache",
        sa.Column("id", sa.Integer(), primary_key=True, autoincrement=True),
        sa.Column("query_hash", sa.String(64), unique=True, nullable=False),
        sa.Column("source_language", sa.String(5), nullable=False),
        sa.Column("target_language", sa.String(5), nullable=False),
        sa.Column("original_query", sa.Text(), nullable=False),
        sa.Column("translated_response", sa.Text(), nullable=False),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            server_default=sa.func.now(),
            nullable=False,
        ),
        sa.Column(
            "last_accessed",
            sa.DateTime(timezone=True),
            server_default=sa.func.now(),
            nullable=False,
        ),
        sa.Column("access_count", sa.Integer(), server_default="1", nullable=False),
    )

    # Create indexes for performance
    # query_hash already has UNIQUE constraint which creates an index
    op.create_index(
        "idx_translation_cache_languages",
        "translation_cache",
        ["source_language", "target_language"],
    )
    op.create_index(
        "idx_translation_cache_last_accessed",
        "translation_cache",
        ["last_accessed"],
    )

    # Add check constraints for supported languages
    op.create_check_constraint(
        "chk_translation_cache_source_lang",
        "translation_cache",
        "source_language IN ('en', 'ur')",
    )
    op.create_check_constraint(
        "chk_translation_cache_target_lang",
        "translation_cache",
        "target_language IN ('en', 'ur')",
    )


def downgrade() -> None:
    """Drop translation_cache table."""
    op.drop_table("translation_cache")
