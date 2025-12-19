"""Guidance generator for manual resolution instructions."""

import logging
from typing import Any

from qa_automation.models.issue import Issue, IssueType, SeverityLevel

logger = logging.getLogger(__name__)


class GuidanceGenerator:
    """
    Generates step-by-step resolution guidance for issues that cannot be auto-fixed.

    Guidance includes:
    1. Issue description and context
    2. Root cause explanation
    3. Recommended solution
    4. Code examples (where applicable)
    5. Testing checklist
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize guidance generator.

        Args:
            config: Configuration dictionary
        """
        self.config = config

    def generate_resolution_guidance(self, issue: Issue) -> str:
        """
        Generate detailed resolution guidance for an issue.

        Args:
            issue: Issue requiring manual resolution

        Returns:
            Formatted guidance text
        """
        guidance = []

        # Header
        guidance.append(f"# Resolution Guidance: {issue.id}")
        guidance.append("")

        # Issue details
        guidance.append("## Issue Details")
        guidance.append(f"- **Type**: {issue.type.value}")
        guidance.append(f"- **Severity**: {issue.severity.value if issue.severity else 'Unknown'}")
        guidance.append(f"- **Location**: {issue.location}")
        guidance.append(f"- **Description**: {issue.description}")
        guidance.append("")

        # Root cause
        if issue.root_cause:
            guidance.append("## Root Cause")
            guidance.append(issue.root_cause)
            guidance.append("")

        # Recommended solution
        guidance.append("## Recommended Solution")
        solution = self._get_solution_template(issue)
        guidance.append(solution)
        guidance.append("")

        # Testing checklist
        guidance.append("## Testing Checklist")
        guidance.append("- [ ] Fix applied in code")
        guidance.append("- [ ] Unit tests pass")
        guidance.append("- [ ] Integration tests pass")
        guidance.append("- [ ] Manual verification completed")
        guidance.append("- [ ] No regressions introduced")
        guidance.append("")

        # Related issues
        if issue.related_issues:
            guidance.append("## Related Issues")
            guidance.append(f"This issue may be related to: {', '.join(issue.related_issues)}")
            guidance.append("")

        # Additional context
        if issue.captured_context:
            guidance.append("## Additional Context")
            for key, value in issue.captured_context.items():
                guidance.append(f"- **{key}**: {value}")
            guidance.append("")

        return "\n".join(guidance)

    def _get_solution_template(self, issue: Issue) -> str:
        """
        Get solution template based on issue type and description.

        Args:
            issue: Issue to generate solution for

        Returns:
            Solution guidance text
        """
        description_lower = issue.description.lower()

        # Authentication issues
        if "authentication" in description_lower or "401" in description_lower:
            return self._auth_solution()

        # 500 errors
        if "500" in description_lower or "internal server error" in description_lower:
            return self._server_error_solution()

        # 404 errors
        if "404" in description_lower or "not found" in description_lower:
            return self._not_found_solution()

        # Performance issues
        if issue.type == IssueType.PERFORMANCE or "slow" in description_lower:
            return self._performance_solution()

        # Frontend console errors
        if "console error" in description_lower or "javascript error" in description_lower:
            return self._js_error_solution()

        # Accessibility issues
        if "accessibility" in description_lower or "alt text" in description_lower:
            return self._accessibility_solution()

        # Generic solution
        return self._generic_solution(issue)

    def _auth_solution(self) -> str:
        return """
1. Verify that the endpoint requires authentication
2. Check that the authentication middleware is applied
3. Ensure tokens/sessions are being passed correctly
4. Verify user has required permissions
5. Check for CORS issues if calling from browser

Example (FastAPI):
```python
@app.get("/api/users", dependencies=[Depends(get_current_user)])
async def list_users(user: User = Depends(get_current_user)):
    return users
```
"""

    def _server_error_solution(self) -> str:
        return """
1. Check server logs for the exact error
2. Add try-catch blocks around the failing code
3. Validate input data before processing
4. Check database connection and queries
5. Verify all required environment variables are set

Example (FastAPI error handling):
```python
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )
```
"""

    def _not_found_solution(self) -> str:
        return """
1. Verify the route is registered in the router
2. Check for typos in the URL path
3. Ensure route parameters match (e.g., user_id vs userId)
4. Check if resource actually exists in database
5. Verify the HTTP method (GET vs POST, etc.)

Example (FastAPI):
```python
@app.get("/api/users/{user_id}")
async def get_user(user_id: int):
    user = await db.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user
```
"""

    def _performance_solution(self) -> str:
        return """
1. Add database indexes for frequently queried fields
2. Implement caching for expensive operations
3. Use pagination for large datasets
4. Optimize database queries (avoid N+1 queries)
5. Consider async operations for I/O-bound tasks

Example (caching):
```python
from functools import lru_cache

@lru_cache(maxsize=100)
def get_expensive_data(key: str):
    # Expensive operation
    return result
```
"""

    def _js_error_solution(self) -> str:
        return """
1. Check browser console for full error stack trace
2. Verify all required imports are present
3. Check for undefined variables or null references
4. Add null checks before accessing properties
5. Use optional chaining (?.) for safer access

Example (TypeScript):
```typescript
// Before
const name = user.profile.name; // Error if profile is null

// After
const name = user?.profile?.name ?? 'Unknown';
```
"""

    def _accessibility_solution(self) -> str:
        return """
1. Add descriptive alt text to all images
2. Ensure proper heading hierarchy (h1, h2, h3)
3. Add ARIA labels where needed
4. Ensure keyboard navigation works
5. Test with screen reader

Example:
```html
<!-- Before -->
<img src="profile.jpg">

<!-- After -->
<img src="profile.jpg" alt="User profile photo">
```
"""

    def _generic_solution(self, issue: Issue) -> str:
        return f"""
1. Review the issue description carefully: {issue.description}
2. Examine the context: {issue.location}
3. Check for similar issues in the codebase
4. Consult relevant documentation
5. Test thoroughly after making changes

Severity: {issue.severity.value if issue.severity else 'Unknown'}
- Critical/High: Fix immediately before deployment
- Medium: Fix in next sprint
- Low: Add to backlog
"""
