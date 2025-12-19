"""Sample FastAPI backend with known issues for testing."""

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel
import time

app = FastAPI(title="Sample Backend - QA Test API")


class User(BaseModel):
    """User model."""
    id: int
    name: str
    email: str


# Issue 1: Slow endpoint (>1s response time)
@app.get("/api/slow")
async def slow_endpoint():
    """Intentionally slow endpoint."""
    time.sleep(2)  # 2 second delay
    return {"message": "This endpoint is slow"}


# Issue 2: 500 error endpoint
@app.get("/api/error")
async def error_endpoint():
    """Intentionally raises 500 error."""
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="Intentional server error for testing"
    )


# Issue 3: Missing validation
@app.post("/api/users")
async def create_user(name: str, email: str):
    """Create user - missing request body validation."""
    # Should use User model for validation
    return {"id": 1, "name": name, "email": email}


# Issue 4: Incorrect status code
@app.delete("/api/users/{user_id}")
async def delete_user(user_id: int):
    """Delete user - returns 200 instead of 204."""
    return {"message": "User deleted"}  # Should return 204 No Content


# Working endpoints
@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok"}


@app.get("/api/users")
async def list_users():
    """List all users."""
    return [
        {"id": 1, "name": "User 1", "email": "user1@example.com"},
        {"id": 2, "name": "User 2", "email": "user2@example.com"},
    ]


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)
