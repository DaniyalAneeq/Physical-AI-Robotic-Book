"""Security utilities for token generation and hashing."""

import hashlib
import hmac
import secrets
from typing import Tuple


def generate_session_token() -> str:
    """
    Generate a cryptographically secure random session token.

    Uses secrets.token_urlsafe() to generate a 32-byte random token
    encoded as URL-safe base64 string.

    Returns:
        str: URL-safe base64-encoded random token (43 characters)

    Example:
        >>> token = generate_session_token()
        >>> len(token)
        43
    """
    return secrets.token_urlsafe(32)


def hash_token(token: str, secret_key: str) -> str:
    """
    Hash a session token using HMAC-SHA256.

    Follows Better Auth session token hashing patterns:
    - Uses HMAC-SHA256 for cryptographic hashing
    - Secret key from environment variable SESSION_SECRET
    - Produces 64-character hex digest

    Args:
        token: Plain session token to hash
        secret_key: Secret key for HMAC (from environment)

    Returns:
        str: HMAC-SHA256 hex digest (64 characters)

    Example:
        >>> token = "abc123xyz"
        >>> secret = "my-secret-key"
        >>> hashed = hash_token(token, secret)
        >>> len(hashed)
        64
    """
    return hmac.new(
        key=secret_key.encode("utf-8"),
        msg=token.encode("utf-8"),
        digestmod=hashlib.sha256,
    ).hexdigest()


def generate_and_hash_token(secret_key: str) -> Tuple[str, str]:
    """
    Generate a session token and its hash in one operation.

    Args:
        secret_key: Secret key for HMAC hashing

    Returns:
        Tuple[str, str]: (plain_token, hashed_token)

    Example:
        >>> secret = "my-secret-key"
        >>> plain, hashed = generate_and_hash_token(secret)
        >>> len(plain)
        43
        >>> len(hashed)
        64
    """
    plain_token = generate_session_token()
    hashed_token = hash_token(plain_token, secret_key)
    return plain_token, hashed_token
