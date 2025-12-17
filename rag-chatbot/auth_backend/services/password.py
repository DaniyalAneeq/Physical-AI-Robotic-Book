"""Password hashing and verification service using Argon2id."""

from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError, VerificationError


class PasswordService:
    """
    Service for password hashing and verification.

    Uses Argon2id algorithm following Better Auth security standards:
    - Memory cost: 65536 KB (64 MB)
    - Time cost: 3 iterations
    - Parallelism: 1 thread

    Better Auth reference:
    - https://www.better-auth.com/docs/concepts/password-hashing
    """

    def __init__(self):
        """Initialize password hasher with Argon2id configuration."""
        self.hasher = PasswordHasher(
            time_cost=3,  # 3 iterations
            memory_cost=65536,  # 64 MB
            parallelism=1,  # 1 thread
            hash_len=32,  # 32-byte hash
            salt_len=16,  # 16-byte salt
        )

    def hash_password(self, password: str) -> str:
        """
        Hash a password using Argon2id.

        Args:
            password: Plain text password to hash

        Returns:
            str: Argon2id hash string (includes salt and parameters)

        Example:
            >>> service = PasswordService()
            >>> hashed = service.hash_password("MySecurePass123")
            >>> hashed.startswith("$argon2id$")
            True
        """
        return self.hasher.hash(password)

    def verify_password(self, password: str, password_hash: str) -> bool:
        """
        Verify a password against its hash.

        Uses constant-time comparison to prevent timing attacks.

        Args:
            password: Plain text password to verify
            password_hash: Argon2id hash to verify against

        Returns:
            bool: True if password matches hash, False otherwise

        Example:
            >>> service = PasswordService()
            >>> hashed = service.hash_password("MySecurePass123")
            >>> service.verify_password("MySecurePass123", hashed)
            True
            >>> service.verify_password("WrongPassword", hashed)
            False
        """
        try:
            self.hasher.verify(password_hash, password)
            return True
        except (VerifyMismatchError, VerificationError):
            return False

    def needs_rehash(self, password_hash: str) -> bool:
        """
        Check if a password hash needs rehashing.

        This happens when Argon2id parameters have been updated.

        Args:
            password_hash: Argon2id hash to check

        Returns:
            bool: True if hash needs rehashing, False otherwise
        """
        return self.hasher.check_needs_rehash(password_hash)


# Singleton instance
password_service = PasswordService()
