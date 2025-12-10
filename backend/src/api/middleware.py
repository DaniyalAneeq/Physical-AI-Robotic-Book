"""Rate limiting and other middleware."""

from collections import defaultdict
from datetime import datetime, timedelta
from typing import Callable
from uuid import UUID

from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware

from ..config import get_settings

settings = get_settings()


class RateLimiter:
    """Token bucket rate limiter per session."""

    def __init__(
        self,
        max_requests: int = 60,
        window_seconds: int = 60,
    ):
        """Initialize rate limiter.

        Args:
            max_requests: Maximum requests allowed in window
            window_seconds: Time window in seconds
        """
        self.max_requests = max_requests
        self.window = timedelta(seconds=window_seconds)
        self.requests: dict[str, list[datetime]] = defaultdict(list)

    def check(self, session_id: str) -> tuple[bool, int]:
        """Check if request is allowed.

        Args:
            session_id: Session identifier

        Returns:
            Tuple of (is_allowed, retry_after_seconds)
        """
        now = datetime.utcnow()

        # Remove expired requests
        self.requests[session_id] = [
            t for t in self.requests[session_id]
            if now - t < self.window
        ]

        if len(self.requests[session_id]) >= self.max_requests:
            # Calculate retry_after
            oldest = min(self.requests[session_id])
            retry_after = int((oldest + self.window - now).total_seconds()) + 1
            return False, max(1, retry_after)

        self.requests[session_id].append(now)
        return True, 0

    def cleanup_old_sessions(self) -> int:
        """Remove entries for sessions with no recent requests.

        Returns:
            Number of sessions cleaned up
        """
        now = datetime.utcnow()
        old_sessions = [
            sid for sid, times in self.requests.items()
            if not times or (now - max(times)) > self.window * 2
        ]
        for sid in old_sessions:
            del self.requests[sid]
        return len(old_sessions)


# Global rate limiter instance
rate_limiter = RateLimiter(
    max_requests=settings.rate_limit_requests,
    window_seconds=settings.rate_limit_window_seconds,
)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware for rate limiting API requests."""

    async def dispatch(self, request: Request, call_next: Callable):
        """Process request with rate limiting."""
        # Only rate limit chat endpoints
        if not request.url.path.startswith("/api/chat"):
            return await call_next(request)

        # Get session ID from request body or header
        session_id = None

        # Try to get from header first
        session_id = request.headers.get("X-Session-ID")

        # If not in header, try to get from query params
        if not session_id:
            session_id = request.query_params.get("session_id")

        # Use client IP as fallback (for initial requests without session)
        if not session_id:
            session_id = request.client.host if request.client else "unknown"

        # Check rate limit
        is_allowed, retry_after = rate_limiter.check(session_id)

        if not is_allowed:
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "error": "rate_limit_exceeded",
                    "message": f"Rate limit of {settings.rate_limit_requests} requests per minute exceeded. Retry after {retry_after} seconds.",
                    "retry_after": retry_after,
                },
                headers={"Retry-After": str(retry_after)},
            )

        return await call_next(request)


def get_session_id_from_request(request: Request) -> str | None:
    """Extract session ID from request.

    Checks header, query params, and body.
    """
    # Header
    session_id = request.headers.get("X-Session-ID")
    if session_id:
        return session_id

    # Query params
    session_id = request.query_params.get("session_id")
    if session_id:
        return session_id

    return None
