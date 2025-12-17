"""
Better Auth Integration for RAG Chatbot

This package implements authentication following Better Auth architectural patterns
adapted for FastAPI and Neon Serverless Postgres.

Features:
- Email/password authentication with Argon2id hashing
- Google OAuth 2.0 signup and login
- Database-backed sessions with cryptographic tokens
- Cookie-based authentication (HttpOnly, Secure, SameSite)
- Protected API endpoints
"""

__version__ = "0.1.0"
