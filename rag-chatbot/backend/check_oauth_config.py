#!/usr/bin/env python3
"""
OAuth Configuration Checker

This script verifies that your OAuth configuration is correct.
Run this before attempting OAuth login to catch configuration errors early.
"""

import sys
from pathlib import Path

# Add parent directory to path to import auth_backend
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from auth_backend.config import settings
    print("✅ Successfully loaded auth_backend config\n")
except Exception as e:
    print(f"❌ Failed to load auth_backend config: {e}")
    sys.exit(1)

print("=" * 60)
print("OAUTH CONFIGURATION CHECK")
print("=" * 60)

# Check Google OAuth credentials
print("\n📍 Google OAuth Configuration:")
print("-" * 60)

if settings.oauth_google_client_id:
    print(f"✅ OAUTH_GOOGLE_CLIENT_ID: {settings.oauth_google_client_id[:20]}...")
else:
    print("❌ OAUTH_GOOGLE_CLIENT_ID: NOT SET")

if settings.oauth_google_client_secret:
    print(f"✅ OAUTH_GOOGLE_CLIENT_SECRET: {settings.oauth_google_client_secret[:10]}...")
else:
    print("❌ OAUTH_GOOGLE_CLIENT_SECRET: NOT SET")

print(f"✅ OAUTH_GOOGLE_REDIRECT_URI: {settings.oauth_google_redirect_uri}")

# Check other important settings
print("\n📍 Other Settings:")
print("-" * 60)
print(f"✅ FRONTEND_URL: {settings.frontend_url}")
print(f"✅ SECURE_COOKIES: {settings.secure_cookies}")
print(f"✅ SAME_SITE_COOKIES: {settings.same_site_cookies}")
print(f"✅ CORS_ORIGINS: {settings.cors_origins}")

# Validation checks
print("\n📍 Validation Checks:")
print("-" * 60)

errors = []
warnings = []

# Check if OAuth is configured
if not settings.oauth_google_client_id or not settings.oauth_google_client_secret:
    errors.append("Google OAuth credentials not configured")

# Check redirect URI format
redirect_uri = settings.oauth_google_redirect_uri
if not redirect_uri.startswith("http://") and not redirect_uri.startswith("https://"):
    errors.append(f"Redirect URI must start with http:// or https://: {redirect_uri}")

if "https://e-book-physical-ai-humanoid-robotics.onrender.com" in redirect_uri or "127.0.0.1:8000" in redirect_uri:
    print("✅ Using localhost redirect URI (development)")
else:
    warnings.append("Redirect URI is not localhost - ensure it's configured in Google Cloud Console")

if not redirect_uri.endswith("/auth/oauth/google/callback"):
    errors.append(f"Redirect URI should end with /auth/oauth/google/callback: {redirect_uri}")

# Check CORS
if "localhost:3000" not in settings.cors_origins and "127.0.0.1:3000" not in settings.cors_origins:
    warnings.append("Frontend URL not in CORS origins - this may cause CORS errors")

# Check secure cookies in production
if settings.secure_cookies and not redirect_uri.startswith("https://"):
    errors.append("SECURE_COOKIES is True but redirect URI uses HTTP (must use HTTPS)")

# Print results
if errors:
    print("\n❌ ERRORS FOUND:")
    for i, error in enumerate(errors, 1):
        print(f"  {i}. {error}")

if warnings:
    print("\n⚠️  WARNINGS:")
    for i, warning in enumerate(warnings, 1):
        print(f"  {i}. {warning}")

if not errors and not warnings:
    print("✅ All checks passed!")

# Print next steps
print("\n" + "=" * 60)
print("NEXT STEPS")
print("=" * 60)

if errors:
    print("\n❌ Fix the errors above before proceeding.")
    print("\nUpdate your .env file:")
    print(f"  File: {Path(__file__).parent / '.env'}")
    print("\nThen restart the backend server.")
    sys.exit(1)

print("\n✅ Configuration looks good!")
print("\nMake sure you've added this redirect URI to Google Cloud Console:")
print(f"\n  {settings.oauth_google_redirect_uri}")
print("\nSteps:")
print("  1. Go to: https://console.cloud.google.com/apis/credentials")
print(f"  2. Find OAuth client: {settings.oauth_google_client_id}")
print("  3. Add authorized redirect URI (exact match required)")
print("  4. Save changes")
print("  5. Wait ~30 seconds for propagation")
print("\nAlternatively, also add these URIs (to handle both localhost and 127.0.0.1):")
print("  - http://localhost:8000/auth/oauth/google/callback")
print("  - http://127.0.0.1:8000/auth/oauth/google/callback")

print("\n" + "=" * 60)
print("\nTo test OAuth flow:")
print("  1. Start backend: uvicorn app.main:app --reload")
print("  2. Start frontend: npm start")
print("  3. Navigate to: http://localhost:3000")
print("  4. Click 'Sign up with Google'")
print("=" * 60)
