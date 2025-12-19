"""
Add i18n support to users table.

Adds preferred_locale column to users table for language preference storage.

Revision ID: 003_add_i18n_support
Revises: 002_add_onboarding
Created: 2025-12-17
"""

from alembic import op
import sqlalchemy as sa

# Revision identifiers
revision = "003_add_i18n_support"
down_revision = "002_add_onboarding"
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Add i18n support to users table."""

    # Add preferred_locale column (nullable first for backfill)
    op.add_column(
        "users",
        sa.Column("preferred_locale", sa.String(5), nullable=True)
    )

    # Backfill existing users with default locale 'en'
    op.execute("UPDATE users SET preferred_locale = 'en' WHERE preferred_locale IS NULL")

    # Make column non-nullable and add default
    op.alter_column(
        "users",
        "preferred_locale",
        nullable=False,
        server_default="en"
    )

    # Add check constraint for supported locales
    op.create_check_constraint(
        "chk_users_locale",
        "users",
        "preferred_locale IN ('en', 'ur')"
    )

    # Add index for analytics queries
    op.create_index(
        "idx_users_preferred_locale",
        "users",
        ["preferred_locale"]
    )


def downgrade() -> None:
    """Remove i18n support from users table."""

    # Drop index
    op.drop_index("idx_users_preferred_locale", table_name="users")

    # Drop check constraint
    op.drop_constraint("chk_users_locale", "users", type_="check")

    # Drop column
    op.drop_column("users", "preferred_locale")
