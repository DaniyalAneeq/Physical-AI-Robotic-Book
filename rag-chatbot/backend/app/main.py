"""FastAPI application entry point for RAG Chatbot."""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api import chatkit, conversations, health, index
from app.config import get_settings

settings = get_settings()

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


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
    title="RAG Chatbot API",
    description="API for the Physical AI & Humanoid Robotics textbook RAG chatbot",
    version="1.0.0",
    lifespan=lifespan,
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
app.include_router(chatkit.router, tags=["ChatKit"])
app.include_router(conversations.router, prefix="/api", tags=["Conversations"])
app.include_router(index.router, prefix="/api", tags=["Indexing"])


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
    }
