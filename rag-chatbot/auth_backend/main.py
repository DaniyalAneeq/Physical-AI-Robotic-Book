"""
⚠️  DEPRECATED - This standalone server is no longer used.

The authentication backend has been merged with the RAG backend into a unified server.

To start the unified server, use:
    cd ../backend
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

The unified server (backend/app/main.py) integrates this auth module and serves both:
    - RAG endpoints: /api/*
    - Auth endpoints: /auth/*

OLD DOCUMENTATION BELOW (for reference only):
================================================================

Main FastAPI application for authentication backend.

This module provides the authentication API that can be integrated
into the main RAG chatbot application.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.sessions import SessionMiddleware

from auth_backend.api.routes import auth, oauth
from auth_backend.config import settings

# Create FastAPI app
app = FastAPI(
    title="Authentication API",
    description="Better Auth-based authentication for RAG Chatbot",
    version="0.1.0",
    docs_url="/api/auth/docs",
    redoc_url="/api/auth/redoc",
)

# Add session middleware for OAuth state management
# This is required by authlib OAuth integration
app.add_middleware(
    SessionMiddleware,
    secret_key=settings.session_secret,
    session_cookie="oauth_state",
    max_age=600,  # OAuth state expires in 10 minutes
    same_site="lax",
    https_only=settings.secure_cookies,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth.router, prefix="/api")
app.include_router(oauth.router, prefix="/api")


@app.get("/api/health")
async def health_check() -> dict:
    """Health check endpoint."""
    return {"status": "healthy", "service": "auth-backend"}


@app.get("/")
async def root() -> dict:
    """Root endpoint with API information."""
    return {
        "service": "Authentication API",
        "version": "0.1.0",
        "docs": "/api/auth/docs",
        "endpoints": {
            "register": "POST /api/auth/register",
            "login": "POST /api/auth/login",
            "logout": "POST /api/auth/logout",
            "session": "GET /api/auth/session",
            "oauth_google": "GET /api/auth/oauth/google",
            "oauth_callback": "GET /api/auth/oauth/google/callback",
        },
    }
