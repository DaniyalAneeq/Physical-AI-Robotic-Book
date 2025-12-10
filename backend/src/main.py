"""FastAPI application entry point."""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import get_settings
from .api.routes import router
from .api.middleware import RateLimitMiddleware
from .services.session import session_service
from .services.retrieval import retrieval_service

settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup and shutdown."""
    # Startup: Initialize connections
    print("Initializing services...")
    await session_service.initialize()
    await retrieval_service.ensure_collection()
    print("Services initialized.")
    yield
    # Shutdown: Close connections
    print("Shutting down services...")
    await session_service.close()
    print("Services shut down.")


app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Integrated RAG Chatbot embedded in the Physical AI Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
)

# Rate limiting middleware (must be added before CORS)
app.add_middleware(RateLimitMiddleware)

# CORS middleware - Note: Wildcard patterns don't work in CORS
# For production, add specific allowed origins
ALLOWED_ORIGINS = [
    "http://localhost:3000",  # Docusaurus dev server
    "http://localhost:8000",  # FastAPI dev server
    "http://127.0.0.1:3000",  # Alternative localhost
    "http://127.0.0.1:8000",  # Alternative localhost
    # GitHub Pages - add your specific GitHub Pages URL
    "https://daniyalaneeq.github.io/Physical-AI-Robotic-Book/",
    # Add any custom domain if using one
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["X-Session-ID"],
)

# Include API routes
app.include_router(router, prefix="/api")
