# Data Model: Better Auth Integration

**Feature**: 001-better-auth
**Date**: 2025-12-14
**Phase**: 1 (Data Design)

## Overview

This document defines the database schema and SQLAlchemy models for the authentication system. The schema follows Better Auth's database patterns adapted for Neon Serverless Postgres.

---

## Entity Relationship Diagram

```
┌─────────────┐
│    users    │
│─────────────│
│ id          │◄──┐
│ email       │   │
│ name        │   │
│ password_hash│   │
│ email_verified_at│ │
│ created_at  │   │
│ updated_at  │   │
└─────────────┘   │
                   │ (1:N)
    ┌──────────────┼─────────────────┐
    │              │                  │
    │              │                  │
┌───▼──────────┐  │              ┌───▼───────────────┐
│   sessions   │  │              │  oauth_accounts   │
│──────────────│  │              │───────────────────│
│ id           │  │              │ id                │
│ user_id      ├──┘              │ user_id           │
│ token_hash   │                 │ provider          │
│ created_at   │                 │ provider_account_id│
│ last_used_at │                 │ access_token      │
│ expires_at   │                 │ refresh_token     │
│ revoked      │                 │ expires_at        │
│ ip_address   │                 │ scope             │
│ user_agent   │                 │ token_type        │
└──────────────┘                 │ created_at        │
                                 │ updated_at        │
                                 └───────────────────┘
```

---

## Database Schema

### 1. `users` Table

**Purpose**: Store user account information for both email/password and OAuth users.

**Columns**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique user identifier |
| `email` | CITEXT | UNIQUE, NOT NULL | User's email address (case-insensitive) |
| `name` | VARCHAR(255) | NOT NULL | User's display name |
| `password_hash` | TEXT | NULLABLE | Argon2id hashed password (null for OAuth-only users) |
| `email_verified_at` | TIMESTAMP WITH TIME ZONE | NULLABLE | Timestamp when email was verified (future feature) |
| `created_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| `updated_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Last account update timestamp |

**Indexes**:
```sql
CREATE UNIQUE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);
```

**Validation Rules**:
- Email must be valid format (validated by Pydantic before insert)
- Email stored in lowercase
- Password hash must never be null for email/password users
- Name must be 1-255 characters

**Sample Data**:
```sql
INSERT INTO users (email, name, password_hash, email_verified_at, created_at, updated_at)
VALUES (
    'alice@example.com',
    'Alice Smith',
    '$argon2id$v=19$m=65536,t=3,p=1$...',  -- Argon2id hash
    NOW(),
    NOW(),
    NOW()
);
```

---

### 2. `sessions` Table

**Purpose**: Store active user sessions for authentication.

**Columns**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique session identifier |
| `user_id` | UUID | NOT NULL, FOREIGN KEY → users.id ON DELETE CASCADE | Reference to user |
| `token_hash` | VARCHAR(64) | UNIQUE, NOT NULL | SHA-256 hash of session token |
| `created_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Session creation timestamp |
| `last_used_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Last activity timestamp (sliding expiration) |
| `expires_at` | TIMESTAMP WITH TIME ZONE | NOT NULL | Session expiration timestamp (30 days from creation) |
| `revoked` | BOOLEAN | NOT NULL, DEFAULT FALSE | Session revocation flag |
| `ip_address` | INET | NULLABLE | IP address of session creation (for audit) |
| `user_agent` | TEXT | NULLABLE | Browser/device user agent (for device management) |

**Indexes**:
```sql
CREATE UNIQUE INDEX idx_sessions_token_hash ON sessions(token_hash);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
CREATE INDEX idx_sessions_active ON sessions(user_id, revoked, expires_at DESC) WHERE revoked = FALSE;
```

**Validation Rules**:
- token_hash must be 64 characters (SHA-256 hex)
- expires_at must be in the future at creation
- last_used_at must be <= expires_at
- Revoked sessions cannot be un-revoked

**State Transitions**:
```
[Created] → (user activity) → [Active with updated last_used_at]
[Active] → (logout or expiration) → [Revoked]
[Active] → (30 days) → [Expired] (automatic via expires_at check)
```

**Sample Data**:
```sql
INSERT INTO sessions (user_id, token_hash, created_at, last_used_at, expires_at, revoked, ip_address, user_agent)
VALUES (
    'a0eebc99-9c0b-4ef8-bb6d-6bb9bd380a11',  -- user_id
    'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',  -- token_hash
    NOW(),
    NOW(),
    NOW() + INTERVAL '30 days',
    FALSE,
    '192.0.2.1'::INET,
    'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
);
```

---

### 3. `oauth_accounts` Table

**Purpose**: Store linked OAuth provider accounts (Google) for users.

**Columns**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique OAuth account identifier |
| `user_id` | UUID | NOT NULL, FOREIGN KEY → users.id ON DELETE CASCADE | Reference to user |
| `provider` | VARCHAR(50) | NOT NULL | OAuth provider name ('google') |
| `provider_account_id` | VARCHAR(255) | NOT NULL | Provider-specific user ID (Google 'sub') |
| `access_token` | TEXT | NULLABLE | OAuth access token (encrypted at rest) |
| `refresh_token` | TEXT | NULLABLE | OAuth refresh token (encrypted at rest) |
| `expires_at` | TIMESTAMP WITH TIME ZONE | NULLABLE | Access token expiration timestamp |
| `scope` | VARCHAR(255) | NULLABLE | OAuth scopes granted |
| `token_type` | VARCHAR(50) | NULLABLE | Token type ('Bearer') |
| `created_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | OAuth account link timestamp |
| `updated_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Last token refresh timestamp |

**Indexes**:
```sql
CREATE INDEX idx_oauth_accounts_user_id ON oauth_accounts(user_id);
CREATE UNIQUE INDEX idx_oauth_accounts_provider ON oauth_accounts(provider, provider_account_id);
```

**Constraints**:
```sql
ALTER TABLE oauth_accounts ADD CONSTRAINT unique_provider_account UNIQUE (provider, provider_account_id);
```

**Validation Rules**:
- provider must be 'google' (initially; expandable for future providers)
- provider_account_id must match Google 'sub' claim format
- Tokens must be encrypted before storage (encryption key in environment)
- Only one OAuth account per provider per user

**Sample Data**:
```sql
INSERT INTO oauth_accounts (user_id, provider, provider_account_id, access_token, refresh_token, expires_at, scope, token_type, created_at, updated_at)
VALUES (
    'a0eebc99-9c0b-4ef8-bb6d-6bb9bd380a11',  -- user_id
    'google',
    '110169484474386276334',  -- Google sub
    'ya29.a0AfH6SMBx...',  -- encrypted access_token
    '1//0gHZiTPFREMsCgYIARAAGBASNwF...',  -- encrypted refresh_token
    NOW() + INTERVAL '1 hour',
    'openid email profile',
    'Bearer',
    NOW(),
    NOW()
);
```

---

## SQLAlchemy Models

### Base Configuration

```python
from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship
from sqlalchemy import String, Text, Boolean, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, INET, CITEXT
import uuid
from datetime import datetime
from typing import Optional, List

class Base(AsyncAttrs, DeclarativeBase):
    """Base class for all models"""
    pass
```

---

### User Model

```python
from sqlalchemy import func

class User(Base):
    __tablename__ = "users"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    email: Mapped[str] = mapped_column(
        CITEXT,  # Case-insensitive text
        unique=True,
        nullable=False,
        index=True
    )
    name: Mapped[str] = mapped_column(
        String(255),
        nullable=False
    )
    password_hash: Mapped[Optional[str]] = mapped_column(
        Text,
        nullable=True  # Null for OAuth-only users
    )
    email_verified_at: Mapped[Optional[datetime]] = mapped_column(
        DateTime(timezone=True),
        nullable=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now(),
        index=True
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now(),
        onupdate=func.now()
    )

    # Relationships
    sessions: Mapped[List["Session"]] = relationship(
        "Session",
        back_populates="user",
        cascade="all, delete-orphan"
    )
    oauth_accounts: Mapped[List["OAuthAccount"]] = relationship(
        "OAuthAccount",
        back_populates="user",
        cascade="all, delete-orphan"
    )

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, name={self.name})>"
```

---

### Session Model

```python
class Session(Base):
    __tablename__ = "sessions"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        nullable=False,
        index=True
    )
    token_hash: Mapped[str] = mapped_column(
        String(64),  # SHA-256 hex = 64 chars
        unique=True,
        nullable=False,
        index=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now()
    )
    last_used_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now()
    )
    expires_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        index=True
    )
    revoked: Mapped[bool] = mapped_column(
        Boolean,
        nullable=False,
        default=False,
        server_default="false"
    )
    ip_address: Mapped[Optional[str]] = mapped_column(
        INET,
        nullable=True
    )
    user_agent: Mapped[Optional[str]] = mapped_column(
        Text,
        nullable=True
    )

    # Relationships
    user: Mapped["User"] = relationship(
        "User",
        back_populates="sessions"
    )

    def is_valid(self) -> bool:
        """Check if session is valid (not revoked and not expired)"""
        return not self.revoked and self.expires_at > datetime.utcnow()

    def __repr__(self):
        return f"<Session(id={self.id}, user_id={self.user_id}, revoked={self.revoked})>"
```

---

### OAuthAccount Model

```python
class OAuthAccount(Base):
    __tablename__ = "oauth_accounts"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        nullable=False,
        index=True
    )
    provider: Mapped[str] = mapped_column(
        String(50),
        nullable=False
    )
    provider_account_id: Mapped[str] = mapped_column(
        String(255),
        nullable=False
    )
    access_token: Mapped[Optional[str]] = mapped_column(
        Text,
        nullable=True
    )
    refresh_token: Mapped[Optional[str]] = mapped_column(
        Text,
        nullable=True
    )
    expires_at: Mapped[Optional[datetime]] = mapped_column(
        DateTime(timezone=True),
        nullable=True
    )
    scope: Mapped[Optional[str]] = mapped_column(
        String(255),
        nullable=True
    )
    token_type: Mapped[Optional[str]] = mapped_column(
        String(50),
        nullable=True
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now()
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        nullable=False,
        server_default=func.now(),
        onupdate=func.now()
    )

    # Relationships
    user: Mapped["User"] = relationship(
        "User",
        back_populates="oauth_accounts"
    )

    # Unique constraint
    __table_args__ = (
        Index(
            "idx_oauth_accounts_provider",
            "provider",
            "provider_account_id",
            unique=True
        ),
    )

    def __repr__(self):
        return f"<OAuthAccount(id={self.id}, provider={self.provider}, user_id={self.user_id})>"
```

---

## Pydantic Schemas (Request/Response)

### User Schemas

```python
from pydantic import BaseModel, EmailStr, Field
from datetime import datetime
from uuid import UUID
from typing import Optional

class UserBase(BaseModel):
    email: EmailStr
    name: str = Field(..., min_length=1, max_length=255)

class UserCreate(UserBase):
    password: str = Field(..., min_length=8, max_length=128)

class UserResponse(UserBase):
    id: UUID
    email_verified_at: Optional[datetime] = None
    created_at: datetime

    class Config:
        from_attributes = True  # SQLAlchemy 2.0 compatibility

class UserWithSession(UserResponse):
    session_id: UUID
    session_expires_at: datetime
```

---

### Session Schemas

```python
class SessionResponse(BaseModel):
    id: UUID
    user_id: UUID
    created_at: datetime
    last_used_at: datetime
    expires_at: datetime
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None

    class Config:
        from_attributes = True

class SessionWithUser(SessionResponse):
    user: UserResponse
```

---

### Auth Request/Response Schemas

```python
class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class RegisterRequest(UserCreate):
    """Same as UserCreate"""
    pass

class AuthResponse(BaseModel):
    user: UserResponse
    session: SessionResponse

class CurrentSessionResponse(BaseModel):
    user: Optional[UserResponse] = None
    session: Optional[SessionResponse] = None
```

---

## Database Migrations

### Alembic Migration Scripts

**Migration 001: Initial Schema**

```python
"""Initial authentication schema

Revision ID: 001_initial_auth
Revises:
Create Date: 2025-12-14
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

def upgrade():
    # Users table
    op.create_table(
        'users',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('email', postgresql.CITEXT(), nullable=False),
        sa.Column('name', sa.String(255), nullable=False),
        sa.Column('password_hash', sa.Text(), nullable=True),
        sa.Column('email_verified_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
    )
    op.create_unique_constraint('users_email_key', 'users', ['email'])
    op.create_index('idx_users_email', 'users', ['email'], unique=True)
    op.create_index('idx_users_created_at', 'users', ['created_at'])

    # Sessions table
    op.create_table(
        'sessions',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('token_hash', sa.String(64), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.Column('last_used_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('revoked', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('ip_address', postgresql.INET(), nullable=True),
        sa.Column('user_agent', sa.Text(), nullable=True),
    )
    op.create_unique_constraint('sessions_token_hash_key', 'sessions', ['token_hash'])
    op.create_index('idx_sessions_token_hash', 'sessions', ['token_hash'], unique=True)
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'])
    op.create_index('idx_sessions_expires_at', 'sessions', ['expires_at'])
    op.create_index('idx_sessions_active', 'sessions', ['user_id', 'revoked', 'expires_at'], postgresql_where=sa.text('revoked = false'))

    # OAuth Accounts table
    op.create_table(
        'oauth_accounts',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('provider', sa.String(50), nullable=False),
        sa.Column('provider_account_id', sa.String(255), nullable=False),
        sa.Column('access_token', sa.Text(), nullable=True),
        sa.Column('refresh_token', sa.Text(), nullable=True),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('scope', sa.String(255), nullable=True),
        sa.Column('token_type', sa.String(50), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
    )
    op.create_index('idx_oauth_accounts_user_id', 'oauth_accounts', ['user_id'])
    op.create_index('idx_oauth_accounts_provider', 'oauth_accounts', ['provider', 'provider_account_id'], unique=True)

def downgrade():
    op.drop_table('oauth_accounts')
    op.drop_table('sessions')
    op.drop_table('users')
```

---

## Data Access Patterns

### Common Queries

```python
# Get user by email
user = await session.execute(
    select(User).where(User.email == email.lower())
)
user = user.scalar_one_or_none()

# Get active sessions for user
sessions = await session.execute(
    select(Session)
    .where(Session.user_id == user_id)
    .where(Session.revoked == False)
    .where(Session.expires_at > datetime.utcnow())
    .order_by(Session.last_used_at.desc())
)
sessions = sessions.scalars().all()

# Find or create OAuth account
oauth_account = await session.execute(
    select(OAuthAccount)
    .where(OAuthAccount.provider == 'google')
    .where(OAuthAccount.provider_account_id == google_sub)
)
oauth_account = oauth_account.scalar_one_or_none()

# Revoke all sessions for user (on password change)
await session.execute(
    update(Session)
    .where(Session.user_id == user_id)
    .where(Session.revoked == False)
    .values(revoked=True)
)
await session.commit()
```

---

## Data Retention and Cleanup

### Automated Cleanup Jobs

**Expired Sessions Cleanup** (run daily):
```sql
DELETE FROM sessions
WHERE expires_at < NOW() - INTERVAL '7 days'
AND revoked = TRUE;
```

**Old Revoked Sessions** (run weekly):
```sql
DELETE FROM sessions
WHERE revoked = TRUE
AND created_at < NOW() - INTERVAL '90 days';
```

---

## Security Considerations

1. **Password Hashes**: Never return password_hash in API responses
2. **Token Hashes**: Never return raw session tokens after creation
3. **OAuth Tokens**: Encrypt access_token and refresh_token at rest
4. **Indexes**: Ensure token_hash and email have unique indexes for fast lookup
5. **Cascading Deletes**: User deletion automatically removes sessions and OAuth accounts

---

**Data Model Complete**: 2025-12-14
**Ready for**: API Contract design
