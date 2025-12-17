"""Unit tests for password hashing service."""

import pytest
from auth_backend.services.password import PasswordService


def test_hash_password():
    """Test password hashing produces valid Argon2id hash."""
    service = PasswordService()
    password = "MySecurePassword123"

    hashed = service.hash_password(password)

    assert hashed is not None
    assert hashed.startswith("$argon2id$")
    assert len(hashed) > 50


def test_verify_password_correct():
    """Test password verification with correct password."""
    service = PasswordService()
    password = "MySecurePassword123"

    hashed = service.hash_password(password)
    result = service.verify_password(password, hashed)

    assert result is True


def test_verify_password_incorrect():
    """Test password verification with incorrect password."""
    service = PasswordService()
    password = "MySecurePassword123"
    wrong_password = "WrongPassword456"

    hashed = service.hash_password(password)
    result = service.verify_password(wrong_password, hashed)

    assert result is False


def test_hash_different_passwords():
    """Test that different passwords produce different hashes."""
    service = PasswordService()
    password1 = "Password123"
    password2 = "Password456"

    hash1 = service.hash_password(password1)
    hash2 = service.hash_password(password2)

    assert hash1 != hash2


def test_same_password_different_hashes():
    """Test that hashing same password twice produces different hashes (due to salt)."""
    service = PasswordService()
    password = "MyPassword123"

    hash1 = service.hash_password(password)
    hash2 = service.hash_password(password)

    assert hash1 != hash2  # Different due to random salt
    assert service.verify_password(password, hash1)
    assert service.verify_password(password, hash2)
