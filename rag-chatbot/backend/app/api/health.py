"""Health check endpoint."""

from datetime import datetime, timezone
from enum import Enum

from fastapi import APIRouter, Depends
from pydantic import BaseModel
from sqlalchemy import text
from sqlalchemy.orm import Session

from app.models.database import get_db

router = APIRouter()


class ServiceStatus(str, Enum):
    """Service health status."""

    UP = "up"
    DOWN = "down"


class HealthStatus(str, Enum):
    """Overall health status."""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class ServicesHealth(BaseModel):
    """Health status of individual services."""

    database: ServiceStatus
    vector_store: ServiceStatus
    openai: ServiceStatus


class HealthResponse(BaseModel):
    """Health check response."""

    status: HealthStatus
    timestamp: datetime
    services: ServicesHealth


async def check_database(db: Session) -> ServiceStatus:
    """Check database connectivity."""
    try:
        db.execute(text("SELECT 1"))
        return ServiceStatus.UP
    except Exception:
        return ServiceStatus.DOWN


async def check_vector_store() -> ServiceStatus:
    """Check Qdrant vector store connectivity."""
    try:
        from app.services.vector_store import get_vector_store

        store = get_vector_store()
        await store.health_check()
        return ServiceStatus.UP
    except Exception:
        return ServiceStatus.DOWN


async def check_openai() -> ServiceStatus:
    """Check OpenAI API connectivity."""
    try:
        from openai import OpenAI

        from app.config import get_settings

        settings = get_settings()
        client = OpenAI(api_key=settings.openai_api_key)
        # Simple models list call to verify API key
        client.models.list()
        return ServiceStatus.UP
    except Exception:
        return ServiceStatus.DOWN


@router.get("/health", response_model=HealthResponse)
async def health_check(db: Session = Depends(get_db)) -> HealthResponse:
    """Check health of all services."""
    database_status = await check_database(db)
    vector_store_status = await check_vector_store()
    openai_status = await check_openai()

    services = ServicesHealth(
        database=database_status,
        vector_store=vector_store_status,
        openai=openai_status,
    )

    # Determine overall status
    statuses = [database_status, vector_store_status, openai_status]
    down_count = sum(1 for s in statuses if s == ServiceStatus.DOWN)

    if down_count == 0:
        overall_status = HealthStatus.HEALTHY
    elif down_count < len(statuses):
        overall_status = HealthStatus.DEGRADED
    else:
        overall_status = HealthStatus.UNHEALTHY

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.now(timezone.utc),
        services=services,
    )
