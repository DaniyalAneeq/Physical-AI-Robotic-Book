"""Apply preferred_locale migration to users table."""

from sqlalchemy import create_engine, text
from app.config import get_settings

settings = get_settings()
engine = create_engine(settings.database_url)

with engine.connect() as conn:
    # Start transaction
    trans = conn.begin()
    try:
        # Check if column already exists
        result = conn.execute(text("""
            SELECT column_name
            FROM information_schema.columns
            WHERE table_name = 'users' AND column_name = 'preferred_locale'
        """))

        if result.fetchone():
            print('ℹ️  preferred_locale column already exists')
        else:
            # Add preferred_locale column
            conn.execute(text('ALTER TABLE users ADD COLUMN preferred_locale VARCHAR(5)'))
            print('✅ Added preferred_locale column')

            # Set default value for existing records
            conn.execute(text("UPDATE users SET preferred_locale = 'en' WHERE preferred_locale IS NULL"))
            print('✅ Set default locale to "en" for existing users')

            # Make column NOT NULL with default
            conn.execute(text("ALTER TABLE users ALTER COLUMN preferred_locale SET NOT NULL"))
            conn.execute(text("ALTER TABLE users ALTER COLUMN preferred_locale SET DEFAULT 'en'"))
            print('✅ Set NOT NULL constraint and default value')

            # Add check constraint
            conn.execute(text("ALTER TABLE users ADD CONSTRAINT chk_users_locale CHECK (preferred_locale IN ('en', 'ur'))"))
            print('✅ Added check constraint for supported locales')

            # Add index
            conn.execute(text('CREATE INDEX idx_users_preferred_locale ON users (preferred_locale)'))
            print('✅ Created index on preferred_locale')

        trans.commit()
        print('\n✅ All migrations applied successfully!')

    except Exception as e:
        trans.rollback()
        print(f'❌ Error: {e}')
        raise
