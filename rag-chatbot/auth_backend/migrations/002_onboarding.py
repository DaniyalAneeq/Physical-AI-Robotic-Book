"""
Onboarding tables migration.

Adds onboarding_completed flag to users table and creates onboarding_profiles table.

Revision ID: 002_onboarding
Depends on: 001_initial_auth
Created: 2025-12-16
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# Revision identifiers
revision = "002_onboarding"
down_revision = "001_initial_auth"
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Add onboarding schema to database."""

    # Add onboarding_completed column to users table
    op.add_column(
        "users",
        sa.Column(
            "onboarding_completed",
            sa.Boolean(),
            nullable=False,
            server_default="false",
        ),
    )

    # Create index on onboarding_completed for efficient filtering
    op.create_index("idx_users_onboarding", "users", ["onboarding_completed"])

    # Create onboarding_profiles table
    op.create_table(
        "onboarding_profiles",
        sa.Column("id", postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
            unique=True,
        ),
        sa.Column("user_type", sa.String(50), nullable=False),
        sa.Column("area_of_interest", sa.String(100), nullable=False),
        sa.Column("experience_level", sa.String(20), nullable=False),
        sa.Column("topics_of_interest", postgresql.JSONB(), nullable=True),
        sa.Column(
            "completed_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.Column(
            "updated_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
            onupdate=sa.func.now(),
        ),
        # Add constraints for enum-like validation
        sa.CheckConstraint(
            "user_type IN ('Student', 'Researcher', 'Teacher', 'Engineer', 'Other')",
            name="user_type_check",
        ),
        sa.CheckConstraint(
            "experience_level IN ('Beginner', 'Intermediate', 'Advanced')",
            name="experience_level_check",
        ),
    )

    # Create indexes for onboarding_profiles
    op.create_index("idx_onboarding_user_id", "onboarding_profiles", ["user_id"])
    op.create_index(
        "idx_onboarding_completed", "onboarding_profiles", ["completed_at"]
    )
    op.create_index("idx_onboarding_user_type", "onboarding_profiles", ["user_type"])


def downgrade() -> None:
    """Remove onboarding schema from database."""
    # Drop onboarding_profiles table and its indexes
    op.drop_table("onboarding_profiles")

    # Drop index and column from users table
    op.drop_index("idx_users_onboarding", table_name="users")
    op.drop_column("users", "onboarding_completed")
