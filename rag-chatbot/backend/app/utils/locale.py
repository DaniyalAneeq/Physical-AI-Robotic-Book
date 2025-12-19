"""Locale parsing and resolution utilities for i18n support."""

import re
from typing import Optional

from fastapi import Request
from sqlalchemy.orm import Session


def parse_accept_language(header: Optional[str]) -> Optional[str]:
    """
    Parse Accept-Language header and return best matching locale.

    Args:
        header: Accept-Language header value (e.g., "ur-PK,en-US;q=0.9,en;q=0.8")

    Returns:
        Best matching locale code ('en' or 'ur'), or None if no match

    Examples:
        >>> parse_accept_language("ur-PK,en-US;q=0.9")
        'ur'
        >>> parse_accept_language("en-US,en;q=0.9")
        'en'
        >>> parse_accept_language("fr-FR,de;q=0.9")
        None
    """
    if not header:
        return None

    # Supported locales
    SUPPORTED_LOCALES = {"en", "ur"}

    # Parse Accept-Language header
    # Format: "locale;q=quality,locale;q=quality,..."
    # Quality defaults to 1.0 if not specified
    locales = []
    for item in header.split(","):
        item = item.strip()
        if ";" in item:
            locale, quality = item.split(";", 1)
            try:
                # Extract quality value (q=0.9 -> 0.9)
                q_value = float(quality.split("=")[1])
            except (IndexError, ValueError):
                q_value = 1.0
        else:
            locale = item
            q_value = 1.0

        # Extract base language code (ur-PK -> ur, en-US -> en)
        base_locale = locale.split("-")[0].lower()

        if base_locale in SUPPORTED_LOCALES:
            locales.append((base_locale, q_value))

    # Sort by quality (descending)
    locales.sort(key=lambda x: x[1], reverse=True)

    # Return highest quality supported locale
    return locales[0][0] if locales else None


def get_user_locale(
    user_id: Optional[str],
    db: Session,
    request: Request,
    default: str = "en"
) -> str:
    """
    Determine user's preferred locale.

    Priority:
    1. Accept-Language header (explicit client preference)
    2. User's stored preference in database
    3. Default locale ('en')

    Args:
        user_id: User ID to look up stored preference
        db: Database session
        request: FastAPI request object
        default: Default locale if no preference found

    Returns:
        Locale code ('en' or 'ur')
    """
    # Priority 1: Check Accept-Language header
    accept_language = request.headers.get("accept-language")
    if accept_language:
        header_locale = parse_accept_language(accept_language)
        if header_locale:
            return header_locale

    # Priority 2: Check user's stored preference
    if user_id and db:
        try:
            # Import here to avoid circular dependency
            from auth_backend.models.user import User

            user = db.query(User).filter(User.id == user_id).first()
            if user and hasattr(user, "preferred_locale") and user.preferred_locale:
                return user.preferred_locale
        except Exception:
            # If user lookup fails, fall through to default
            pass

    # Priority 3: Default locale
    return default


def validate_locale(locale: str) -> bool:
    """
    Validate that locale is supported.

    Args:
        locale: Locale code to validate

    Returns:
        True if locale is supported, False otherwise
    """
    SUPPORTED_LOCALES = {"en", "ur"}
    return locale in SUPPORTED_LOCALES
