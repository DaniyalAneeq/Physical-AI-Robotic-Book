# Data Model: Authentication Refactor & Integration

**Feature**: 001-auth-refactor
**Date**: 2025-12-16
**Phase**: Phase 1 - Design & Contracts

## Overview

This document defines the data entities, relationships, and validation rules for the authentication refactor feature. The data model extends the existing authentication schema with onboarding profile capabilities while preserving all existing RAG chatbot entities.

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Data Model Overview                              │
└─────────────────────────────────────────────────────────────────────┘

   ┌──────────────┐
   │    User      │◄─────────────────┐
   │              │                  │
   │ - id (PK)    │                  │
   │ - email      │                  │
   │ - name       │                  │
   │ - password   │                  │
   │ - onboarding │                  │
   │   _completed │                  │
   └───────┬──────┘                  │
           │                         │
           │ 1                       │
           │                         │
           │                         │
    ┌──────┴────────┐         ┌──────┴──────────┐
    │               │         │                 │
    │ N             │ N       │ 1               │
    │               │         │                 │
┌───▼────────┐ ┌───▼────────┐ ┌────▼──────────────┐
│  Session   │ │OAuth       │ │Onboarding         │
│            │ │Account     │ │Profile            │
│ - id       │ │            │ │                   │
│ - user_id  │ │ - id       │ │ - id              │
│ - token    │ │ - user_id  │ │ - user_id (UNIQUE)│
│ - expires  │ │ - provider │ │ - user_type       │
└────────────┘ │ - provider │ │ - area_interest   │
               │   _user_id │ │ - experience      │
               └────────────┘ │ - topics[]        │
                              └───────────────────┘

           ┌──────────────┐
           │ Conversation │ (Existing - RAG Chatbot)
           │              │
           │ - id         │
           │ - user_id    │◄───── Links to User
           │ - messages   │
           └──────────────┘
```

## Entities

### 1. User (Modified - existing table)

**Table**: `users`

**Description**: Represents an authenticated user account. Extended with onboarding completion flag.

**Attributes**:

| Field                | Type                 | Constraints                 | Description                                    |
|---------------------|----------------------|-----------------------------|------------------------------------------------|
| id                  | UUID                 | PRIMARY KEY, DEFAULT uuid() | Unique user identifier                         |
| email               | VARCHAR(255)         | UNIQUE, NOT NULL           | User's email address (login identifier)        |
| name                | VARCHAR(255)         | NULL                        | User's display name                            |
| password_hash       | VARCHAR(255)         | NULL                        | Argon2id hashed password (NULL for OAuth-only) |
| email_verified      | BOOLEAN              | DEFAULT FALSE               | Email verification status                      |
| onboarding_completed| BOOLEAN              | DEFAULT FALSE, NOT NULL     | 🆕 Onboarding completion flag                   |
| created_at          | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL     | Account creation timestamp                     |
| updated_at          | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL     | Last update timestamp                          |

**Indexes**:
- `PRIMARY KEY (id)`
- `UNIQUE INDEX idx_users_email ON users(email)`
- `INDEX idx_users_onboarding ON users(onboarding_completed)` 🆕

**Relationships**:
- Has many `sessions` (1:N)
- Has many `oauth_accounts` (1:N)
- Has one `onboarding_profile` (1:1)
- Has many `conversations` (1:N) - Existing RAG relationship

**Validation Rules**:
- `email`: Must be valid email format, max 255 characters
- `password_hash`: Required for email/password accounts, NULL for OAuth-only accounts
- `name`: Optional, max 255 characters
- `onboarding_completed`: Must be FALSE on account creation

**State Transitions**:
```
[New User] → onboarding_completed = FALSE
         ↓
[Complete Onboarding] → onboarding_completed = TRUE
         ↓
[Can Access Application]
```

---

### 2. OnboardingProfile (New table)

**Table**: `onboarding_profiles`

**Description**: Stores user profile data collected during the mandatory onboarding flow.

**Attributes**:

| Field               | Type                 | Constraints                       | Description                                    |
|--------------------|----------------------|-----------------------------------|------------------------------------------------|
| id                 | UUID                 | PRIMARY KEY, DEFAULT uuid()       | Unique profile identifier                      |
| user_id            | UUID                 | FOREIGN KEY users(id), UNIQUE, NOT NULL | Link to user (one profile per user)      |
| user_type          | VARCHAR(50)          | NOT NULL                          | User category (enum-like)                      |
| area_of_interest   | VARCHAR(100)         | NOT NULL                          | Primary area of interest                       |
| experience_level   | VARCHAR(20)          | NOT NULL                          | Skill level (enum-like)                        |
| topics_of_interest | JSONB                | NULL                              | Array of selected topics (optional)            |
| completed_at       | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Onboarding completion timestamp                |
| created_at         | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Record creation timestamp                      |
| updated_at         | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Last update timestamp                          |

**Indexes**:
- `PRIMARY KEY (id)`
- `UNIQUE INDEX idx_onboarding_user_id ON onboarding_profiles(user_id)`
- `INDEX idx_onboarding_completed ON onboarding_profiles(completed_at)`
- `INDEX idx_onboarding_user_type ON onboarding_profiles(user_type)`

**Relationships**:
- Belongs to one `user` (N:1)

**Validation Rules**:
- `user_id`: Must reference existing user, one profile per user
- `user_type`: ENUM constraint ["Student", "Researcher", "Teacher", "Engineer", "Other"]
- `area_of_interest`: Must be one of predefined options (validated at application level)
- `experience_level`: ENUM constraint ["Beginner", "Intermediate", "Advanced"]
- `topics_of_interest`: Optional array, valid values: ["ROS2", "NVIDIA Isaac", "Voice Control", "LLM Integration", "Reinforcement Learning"]

**Example Data**:
```json
{
    "id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "user_id": "a1b2c3d4-1234-5678-9012-abcdef123456",
    "user_type": "Researcher",
    "area_of_interest": "Robotics & Automation",
    "experience_level": "Intermediate",
    "topics_of_interest": ["ROS2", "NVIDIA Isaac", "Reinforcement Learning"],
    "completed_at": "2025-12-16T10:30:00Z",
    "created_at": "2025-12-16T10:30:00Z",
    "updated_at": "2025-12-16T10:30:00Z"
}
```

---

### 3. Session (Existing - no changes)

**Table**: `sessions`

**Description**: Represents an active user authentication session.

**Attributes**:

| Field          | Type                 | Constraints                       | Description                                    |
|---------------|----------------------|-----------------------------------|------------------------------------------------|
| id            | UUID                 | PRIMARY KEY, DEFAULT uuid()       | Unique session identifier                      |
| user_id       | UUID                 | FOREIGN KEY users(id), NOT NULL   | Link to user                                   |
| token_hash    | VARCHAR(64)          | UNIQUE, NOT NULL                  | HMAC-SHA256 hashed session token               |
| expires_at    | TIMESTAMP WITH TZ    | NOT NULL                          | Absolute expiration (30 days)                  |
| last_used_at  | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Last activity timestamp (sliding window)       |
| ip_address    | VARCHAR(45)          | NULL                              | Client IP address                              |
| user_agent    | TEXT                 | NULL                              | Client user agent                              |
| created_at    | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Session creation timestamp                     |

**Relationships**:
- Belongs to one `user` (N:1)

**Validation Rules**:
- `token_hash`: Unique, 64-character hex string
- `expires_at`: Must be in the future when session is created
- `last_used_at`: Updated on every request (sliding session)

---

### 4. OAuthAccount (Existing - no changes)

**Table**: `oauth_accounts`

**Description**: Represents a linked OAuth provider account (Google, GitHub).

**Attributes**:

| Field               | Type                 | Constraints                       | Description                                    |
|--------------------|----------------------|-----------------------------------|------------------------------------------------|
| id                 | UUID                 | PRIMARY KEY, DEFAULT uuid()       | Unique OAuth account identifier                |
| user_id            | UUID                 | FOREIGN KEY users(id), NOT NULL   | Link to user                                   |
| provider           | VARCHAR(50)          | NOT NULL                          | OAuth provider (google, github)                |
| provider_user_id   | VARCHAR(255)         | NOT NULL                          | Provider's user ID                             |
| access_token       | TEXT                 | NULL                              | OAuth access token (encrypted recommended)     |
| refresh_token      | TEXT                 | NULL                              | OAuth refresh token (encrypted recommended)    |
| token_expires_at   | TIMESTAMP WITH TZ    | NULL                              | Access token expiration                        |
| created_at         | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Account linkage timestamp                      |
| updated_at         | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Last token refresh timestamp                   |

**Indexes**:
- `PRIMARY KEY (id)`
- `UNIQUE INDEX idx_oauth_provider_user ON oauth_accounts(provider, provider_user_id)`
- `INDEX idx_oauth_user_id ON oauth_accounts(user_id)`

**Relationships**:
- Belongs to one `user` (N:1)

**Validation Rules**:
- `provider`: ENUM constraint ["google", "github"]
- `provider_user_id`: Required, unique per provider
- Compound uniqueness: (provider, provider_user_id) must be unique

---

### 5. Conversation (Existing - RAG Chatbot - no changes)

**Table**: `conversations`

**Description**: Represents a RAG chatbot conversation session.

**Attributes**:

| Field          | Type                 | Constraints                       | Description                                    |
|---------------|----------------------|-----------------------------------|------------------------------------------------|
| id            | UUID                 | PRIMARY KEY, DEFAULT uuid()       | Unique conversation identifier                 |
| user_id       | UUID                 | FOREIGN KEY users(id), NULL       | Link to user (NULL for anonymous - DEPRECATED) |
| title         | VARCHAR(255)         | NULL                              | Conversation title                             |
| created_at    | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Conversation start timestamp                   |
| updated_at    | TIMESTAMP WITH TZ    | DEFAULT NOW(), NOT NULL           | Last message timestamp                         |

**Relationships**:
- Belongs to one `user` (N:1)
- Has many `messages` (1:N)

**Note**: After authentication refactor, `user_id` will be required (NOT NULL). Existing anonymous conversations will be migrated or archived.

---

## Predefined Values (Enumerations)

### User Types
```python
USER_TYPES = [
    "Student",
    "Researcher",
    "Teacher",
    "Engineer",
    "Other"
]
```

### Areas of Interest
```python
AREAS_OF_INTEREST = [
    "Robotics & Automation",
    "Artificial Intelligence & Machine Learning",
    "Computer Vision",
    "Natural Language Processing",
    "Hardware & Embedded Systems",
    "Simulation & Virtual Environments",
    "Other"
]
```

### Experience Levels
```python
EXPERIENCE_LEVELS = [
    "Beginner",
    "Intermediate",
    "Advanced"
]
```

### Topics of Interest (Multi-select)
```python
TOPICS_OF_INTEREST = [
    "ROS2",
    "NVIDIA Isaac",
    "Voice Control",
    "LLM Integration",
    "Reinforcement Learning",
    "Computer Vision",
    "Path Planning",
    "Manipulation"
]
```

## Database Migrations

### Migration 002: Add Onboarding Schema

**File**: `auth_backend/migrations/002_onboarding.py`

**Up Migration**:
```python
async def upgrade():
    # Add onboarding_completed column to users table
    await conn.execute("""
        ALTER TABLE users
        ADD COLUMN onboarding_completed BOOLEAN NOT NULL DEFAULT FALSE;
    """)

    await conn.execute("""
        CREATE INDEX idx_users_onboarding
        ON users(onboarding_completed);
    """)

    # Create onboarding_profiles table
    await conn.execute("""
        CREATE TABLE onboarding_profiles (
            id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
            user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
            user_type VARCHAR(50) NOT NULL,
            area_of_interest VARCHAR(100) NOT NULL,
            experience_level VARCHAR(20) NOT NULL,
            topics_of_interest JSONB,
            completed_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
            created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
            CONSTRAINT user_type_check CHECK (user_type IN ('Student', 'Researcher', 'Teacher', 'Engineer', 'Other')),
            CONSTRAINT experience_level_check CHECK (experience_level IN ('Beginner', 'Intermediate', 'Advanced'))
        );
    """)

    await conn.execute("""
        CREATE INDEX idx_onboarding_user_id ON onboarding_profiles(user_id);
    """)

    await conn.execute("""
        CREATE INDEX idx_onboarding_completed ON onboarding_profiles(completed_at);
    """)

    await conn.execute("""
        CREATE INDEX idx_onboarding_user_type ON onboarding_profiles(user_type);
    """)
```

**Down Migration**:
```python
async def downgrade():
    await conn.execute("DROP TABLE IF EXISTS onboarding_profiles CASCADE;")
    await conn.execute("DROP INDEX IF EXISTS idx_users_onboarding;")
    await conn.execute("ALTER TABLE users DROP COLUMN IF EXISTS onboarding_completed;")
```

## Data Access Patterns

### Common Queries

**1. Get User with Onboarding Status**:
```sql
SELECT
    u.id,
    u.email,
    u.name,
    u.onboarding_completed,
    op.user_type,
    op.experience_level
FROM users u
LEFT JOIN onboarding_profiles op ON op.user_id = u.id
WHERE u.id = $1;
```

**2. Check Onboarding Requirement**:
```sql
SELECT onboarding_completed
FROM users
WHERE id = $1;
```

**3. Create Onboarding Profile**:
```sql
INSERT INTO onboarding_profiles (
    user_id, user_type, area_of_interest,
    experience_level, topics_of_interest
)
VALUES ($1, $2, $3, $4, $5::jsonb)
RETURNING *;

-- Also update user flag
UPDATE users
SET onboarding_completed = TRUE
WHERE id = $1;
```

**4. Get Onboarding Statistics** (Analytics):
```sql
SELECT
    user_type,
    experience_level,
    COUNT(*) as count
FROM onboarding_profiles
GROUP BY user_type, experience_level
ORDER BY count DESC;
```

**5. Find Users by Interest**:
```sql
SELECT u.id, u.email, u.name
FROM users u
JOIN onboarding_profiles op ON op.user_id = u.id
WHERE op.topics_of_interest ? 'ROS2';  -- JSONB contains operator
```

## Validation Summary

| Entity              | Required Fields                                      | Optional Fields          | Unique Constraints        |
|--------------------|------------------------------------------------------|--------------------------|---------------------------|
| User               | id, email, onboarding_completed, created_at          | name, password_hash      | email                     |
| OnboardingProfile  | id, user_id, user_type, area_interest, experience, completed_at | topics_of_interest | user_id                   |
| Session            | id, user_id, token_hash, expires_at, last_used_at    | ip_address, user_agent   | token_hash                |
| OAuthAccount       | id, user_id, provider, provider_user_id              | access_token, refresh_token | (provider, provider_user_id) |
| Conversation       | id, created_at                                       | user_id, title           | -                         |

## Next Steps

1. **Generate API Contracts**: Define OpenAPI schemas for all authentication and onboarding endpoints
2. **Implement SQLAlchemy Models**: Create Python ORM models for `OnboardingProfile`
3. **Create Pydantic Schemas**: Define request/response schemas for validation
4. **Write Database Tests**: Unit tests for all data access patterns
