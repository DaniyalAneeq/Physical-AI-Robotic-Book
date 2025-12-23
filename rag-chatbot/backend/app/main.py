"""FastAPI application entry point for RAG Chatbot."""

import logging
import sys
from contextlib import asynccontextmanager
from pathlib import Path

# Add parent directory to Python path to find auth_backend module (MUST be before other imports)
parent_dir = Path(__file__).resolve().parent.parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.sessions import SessionMiddleware

from app.api import chatkit, content, conversations, health, index, sessions, user
from app.config import get_settings

settings = get_settings()

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Import auth routers from auth_backend
try:

    from auth_backend.api.routes import auth, oauth, onboarding
    AUTH_AVAILABLE = True
    logger.info("Auth backend loaded successfully")
except ImportError as e:
    AUTH_AVAILABLE = False
    logger.warning(f"Auth backend not available - authentication endpoints disabled: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup and shutdown."""
    # Startup
    logger.info("Starting RAG Chatbot backend...")
    logger.info(f"Environment: {settings.environment}")

    yield

    # Shutdown - gracefully handle in-flight requests
    logger.info("Shutting down RAG Chatbot backend...")
    logger.info("Waiting for in-flight requests to complete...")
    # FastAPI automatically waits for in-flight requests to complete
    logger.info("Shutdown complete")


app = FastAPI(
    title="Unified Backend API",
    description="Unified backend for Physical AI & Humanoid Robotics textbook - RAG chatbot and authentication",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)

# Add session middleware for OAuth state management (required by authlib)
app.add_middleware(
    SessionMiddleware,
    secret_key=settings.session_secret,
    session_cookie="oauth_state",
    max_age=600,  # OAuth state expires in 10 minutes
    same_site="none",
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
app.include_router(health.router, tags=["System"])
app.include_router(chatkit.router, prefix="/api", tags=["ChatKit"])
app.include_router(sessions.router, prefix="/api", tags=["Sessions"])  # Frontend-compatible endpoints
app.include_router(conversations.router, prefix="/api", tags=["Conversations"])
app.include_router(index.router, prefix="/api", tags=["Indexing"])
app.include_router(user.router, tags=["User"])
app.include_router(content.router, tags=["Content"])

# Include auth routers if available (with /auth prefix for authentication endpoints)
if AUTH_AVAILABLE:
    app.include_router(auth.router, prefix="/auth", tags=["Authentication"])
    app.include_router(oauth.router, prefix="/auth/oauth", tags=["OAuth"])
    app.include_router(onboarding.router, prefix="/auth/onboarding", tags=["Onboarding"])
    logger.info("Auth routes registered: /auth/*, /auth/oauth/*, /auth/onboarding/*")


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "Unified Backend API",
        "version": "1.0.0",
        "description": "RAG Chatbot + Authentication",
        "endpoints": {
            "rag": "/api/*",
            "auth": "/auth/*",
            "oauth": "/auth/oauth/*",
        },
        "docs": "/docs",
        "health": "/api/health",
    }
