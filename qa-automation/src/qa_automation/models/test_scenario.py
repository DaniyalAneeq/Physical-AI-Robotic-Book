"""Test scenario entity model."""

from enum import Enum

from pydantic import BaseModel, Field


class TestType(str, Enum):
    """Type of test scenario."""

    FRONTEND = "frontend"
    BACKEND = "backend"
    PERFORMANCE = "performance"


class AuthMethod(str, Enum):
    """Authentication method for test."""

    NONE = "none"
    BETTER_AUTH = "better_auth"
    OAUTH2 = "oauth2"
    JWT_BEARER = "jwt_bearer"
    SESSION_COOKIE = "session_cookie"


class TestStep(BaseModel):
    """
    Individual step in a test scenario.

    Attributes:
        action: Action type (wait_for_selector, click, fill, screenshot, etc.)
        selector: CSS selector for element
        value: Value for fill actions
        timeout: Timeout in milliseconds
    """

    action: str = Field(
        ...,
        description="Action type: wait_for_selector, click, fill, screenshot, etc.",
    )
    selector: str | None = Field(None, description="CSS selector for element")
    value: str | None = Field(None, description="Value for fill actions")
    timeout: int = Field(5000, description="Timeout in milliseconds")


class Assertion(BaseModel):
    """
    Assertion to verify in test.

    Attributes:
        selector: CSS selector for element
        text_contains: Expected text content
        status_code: Expected HTTP status code
        response_schema: JSON schema reference for response
    """

    selector: str | None = None
    text_contains: str | None = None
    status_code: int | None = None
    response_schema: str | None = None


class TestScenario(BaseModel):
    """
    Test scenario entity defining a specific test case to execute.

    Attributes:
        id: Unique scenario identifier
        name: Human-readable name
        type: Type of test (frontend/backend/performance)
        enabled: Whether scenario is enabled
        url: URL for frontend tests or API endpoint for backend tests
        method: HTTP method for backend tests (GET, POST, etc.)
        auth_required: Whether authentication is required
        auth_method: Authentication method to use
        headers: HTTP headers for backend tests
        body: Request body for backend tests
        steps: Steps for frontend tests
        assertions: Assertions to verify
        performance_thresholds: Performance limits (max response time, bundle size, etc.)
    """

    id: str = Field(..., description="Unique scenario identifier")
    name: str = Field(..., min_length=5, max_length=200)
    type: TestType
    enabled: bool = Field(True, description="Whether scenario is enabled")
    url: str | None = Field(
        None,
        description="URL for frontend tests or API endpoint for backend tests",
    )
    method: str | None = Field(
        None, description="HTTP method for backend tests (GET, POST, etc.)"
    )
    auth_required: bool = Field(False)
    auth_method: AuthMethod = AuthMethod.NONE
    headers: dict[str, str] = Field(default_factory=dict)
    body: dict | None = Field(None, description="Request body for backend tests")
    steps: list[TestStep] = Field(
        default_factory=list, description="Steps for frontend tests"
    )
    assertions: list[Assertion] = Field(default_factory=list)
    performance_thresholds: dict = Field(
        default_factory=dict,
        description="Max response time, bundle size, etc.",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "id": "homepage-load",
                "name": "Homepage loads successfully",
                "type": "frontend",
                "enabled": True,
                "url": "http://localhost:3000",
                "method": None,
                "auth_required": False,
                "auth_method": "none",
                "steps": [
                    {
                        "action": "wait_for_selector",
                        "selector": "h1",
                        "timeout": 5000,
                    },
                    {"action": "screenshot", "value": "homepage"},
                ],
                "assertions": [{"selector": "h1", "text_contains": "Welcome"}],
                "performance_thresholds": {"max_page_load_ms": 3000},
            }
        }
