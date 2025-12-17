"""
Initial authentication tables migration.

Creates users, sessions, and oauth_accounts tables.

Revision ID: 001_initial_auth
Created: 2025-12-14
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# Revision identifiers
revision = "001_initial_auth"
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Create authentication tables."""

    # Create users table
    op.create_table(
        "users",
        sa.Column("id", postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column("email", sa.String(255), nullable=False, unique=True),
        sa.Column("name", sa.String(255), nullable=False),
        sa.Column("password_hash", sa.Text(), nullable=True),
        sa.Column(
            "email_verified_at",
            sa.DateTime(timezone=True),
            nullable=True,
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
    )

    # Create indexes for users
    op.create_index("idx_users_email", "users", ["email"])
    op.create_index("idx_users_created_at", "users", ["created_at"])

    # Create auth_sessions table
    op.create_table(
        "auth_sessions",
        sa.Column("id", postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("token_hash", sa.String(64), nullable=False, unique=True),
        sa.Column(
            "last_used_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.Column(
            "expires_at",
            sa.DateTime(timezone=True),
            nullable=False,
        ),
        sa.Column("revoked", sa.Boolean(), nullable=False, server_default="false"),
        sa.Column("ip_address", sa.String(45), nullable=True),
        sa.Column("user_agent", sa.Text(), nullable=True),
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
    )

    # Create indexes for auth_sessions
    op.create_index("idx_auth_sessions_token_hash", "auth_sessions", ["token_hash"])
    op.create_index("idx_auth_sessions_user_id", "auth_sessions", ["user_id"])
    op.create_index("idx_auth_sessions_expires_at", "auth_sessions", ["expires_at"])

    # Create oauth_accounts table
    op.create_table(
        "oauth_accounts",
        sa.Column("id", postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("provider", sa.String(50), nullable=False),
        sa.Column("provider_account_id", sa.String(255), nullable=False),
        sa.Column("access_token", sa.Text(), nullable=True),
        sa.Column("refresh_token", sa.Text(), nullable=True),
        sa.Column("expires_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("scope", sa.Text(), nullable=True),
        sa.Column("token_type", sa.String(50), nullable=True),
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
    )

    # Create indexes for oauth_accounts
    op.create_index("idx_oauth_user_id", "oauth_accounts", ["user_id"])
    op.create_index(
        "idx_oauth_provider_account",
        "oauth_accounts",
        ["provider", "provider_account_id"],
        unique=True,
    )


def downgrade() -> None:
    """Drop authentication tables."""
    op.drop_table("oauth_accounts")
    op.drop_table("auth_sessions")
    op.drop_table("users")
