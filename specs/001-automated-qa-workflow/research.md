# Research: Automated Quality Assurance Workflow

**Date**: 2025-12-17
**Feature**: 001-automated-qa-workflow
**Purpose**: Document technology decisions, best practices, and architectural patterns

---

## 1. Playwright Best Practices for SPA Testing

### Decision: Use Playwright for Browser Automation

**Rationale**:
- Modern, fast, and reliable browser automation framework
- Built-in support for multiple browsers (Chromium, Firefox, WebKit)
- Excellent TypeScript/Python support with type hints
- Built-in tracing and debugging capabilities
- Auto-waiting for elements (reduces flakiness)
- Native support for modern web features (Shadow DOM, iframes, Web Components)

**Alternatives Considered**:
- **Selenium**: Older, slower, more flaky, lacks auto-waiting
- **Cypress**: JavaScript-only, limited multi-browser support, runs in-browser
- **Puppeteer**: Chromium-only, less robust than Playwright

### SPA Testing Patterns

#### 1. **Page Object Model (POM)**
```python
# Best practice: Encapsulate page interactions in page objects
class DashboardPage:
    def __init__(self, page: Page):
        self.page = page
        self.header = page.locator("header.dashboard")
        self.user_menu = page.locator("[data-testid='user-menu']")

    async def navigate(self):
        await self.page.goto("/dashboard")
        await self.header.wait_for()  # Auto-wait for page load

    async def get_user_name(self) -> str:
        return await self.user_menu.text_content()
```

**Why**: Improves maintainability, reduces code duplication, makes tests more readable

#### 2. **Auto-Waiting Strategy**
```python
# Playwright auto-waits - no explicit waits needed
await page.click("button")  # Waits for button to be visible, enabled, stable

# For custom conditions, use expect with timeout
from playwright.sync_api import expect
await expect(page.locator(".loading")).to_be_hidden(timeout=10000)
```

**Why**: Reduces flakiness, eliminates sleep statements, makes tests faster

#### 3. **Artifact Collection**
```python
# Enable tracing for debugging
context = await browser.new_context()
await context.tracing.start(screenshots=True, snapshots=True)

# Take screenshots on failure
try:
    await page.goto("/dashboard")
except Exception:
    await page.screenshot(path="artifacts/failure.png")

# Stop tracing and save
await context.tracing.stop(path="artifacts/trace.zip")
```

**Why**: Essential for debugging failures, especially in CI/CD environments

#### 4. **Parallelization**
```python
# Run tests in parallel with asyncio
async def run_tests_parallel(scenarios):
    browser = await playwright.chromium.launch()
    tasks = [run_single_test(browser, scenario) for scenario in scenarios]
    results = await asyncio.gather(*tasks, return_exceptions=True)
    await browser.close()
    return results
```

**Why**: Significantly reduces test execution time

### References
- Playwright Python Documentation: https://playwright.dev/python/docs/intro
- Playwright Best Practices: https://playwright.dev/docs/best-practices
- Auto-waiting Guide: https://playwright.dev/docs/actionability

---

## 2. httpx Async Patterns for FastAPI Testing

### Decision: Use httpx AsyncClient for API Testing

**Rationale**:
- Full async/await support (essential for parallel testing)
- HTTP/2 support
- Excellent performance for concurrent requests
- Compatible with pytest-asyncio
- Similar API to requests (easy migration)
- Built-in connection pooling

**Alternatives Considered**:
- **requests**: Synchronous only, no HTTP/2, slower for concurrent tests
- **aiohttp**: More complex API, less compatible with FastAPI testing patterns
- **TestClient (Starlette)**: Good for unit tests, but doesn't test real HTTP stack

### Async Testing Patterns

#### 1. **Shared Client with Connection Pooling**
```python
# Reuse client across tests for performance
@pytest.fixture(scope="session")
async def http_client():
    async with httpx.AsyncClient(
        base_url="http://localhost:8000",
        timeout=30.0,
        limits=httpx.Limits(max_connections=100, max_keepalive_connections=20)
    ) as client:
        yield client
```

**Why**: Connection pooling dramatically improves performance for multiple requests

#### 2. **Parallel Request Execution**
```python
# Execute multiple API calls in parallel
async def test_multiple_endpoints(client: httpx.AsyncClient):
    results = await asyncio.gather(
        client.get("/api/users"),
        client.get("/api/posts"),
        client.get("/api/comments"),
        return_exceptions=True
    )

    # Handle individual failures without stopping all tests
    for result in results:
        if isinstance(result, Exception):
            log_error(result)
```

**Why**: Reduces test execution time from sequential to parallel

#### 3. **Request/Response Logging**
```python
# Log all requests and responses for debugging
class LoggingTransport(httpx.AsyncHTTPTransport):
    async def handle_async_request(self, request):
        logger.info(f"REQUEST: {request.method} {request.url}")
        response = await super().handle_async_request(request)
        logger.info(f"RESPONSE: {response.status_code} ({response.elapsed}ms)")
        return response

client = httpx.AsyncClient(transport=LoggingTransport())
```

**Why**: Essential for debugging API test failures

#### 4. **Retry Logic for Flaky Endpoints**
```python
# Retry failed requests with exponential backoff
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
async def call_api_with_retry(client, endpoint):
    response = await client.get(endpoint)
    response.raise_for_status()
    return response
```

**Why**: Handles transient failures (network issues, rate limits)

### References
- httpx Documentation: https://www.python-httpx.org/
- Async Client Guide: https://www.python-httpx.org/async/
- FastAPI Testing: https://fastapi.tiangolo.com/tutorial/testing/

---

## 3. OpenAPI Schema Validation

### Decision: Use jsonschema + openapi-spec-validator

**Rationale**:
- **jsonschema**: Standard Python library for JSON Schema validation
- **openapi-spec-validator**: Validates OpenAPI specs and responses against schemas
- Mature, well-maintained, widely used
- Supports OpenAPI 3.0/3.1 specifications
- Detailed error messages for validation failures

**Alternatives Considered**:
- **pydantic**: Good for Python models, but less suited for dynamic OpenAPI validation
- **openapi-core**: More complex, heavier dependency
- **schemathesis**: Property-based testing, overkill for this use case

### Schema Validation Patterns

#### 1. **Loading OpenAPI Spec**
```python
import yaml
from openapi_spec_validator import validate_spec
from openapi_spec_validator.readers import read_from_filename

# Load and validate OpenAPI spec
spec_dict, spec_url = read_from_filename("openapi.yaml")
validate_spec(spec_dict)  # Raises exception if invalid
```

**Why**: Ensures OpenAPI spec itself is valid before using it for validation

#### 2. **Validating API Responses**
```python
from openapi_core import Spec
from openapi_core.contrib.requests import RequestsOpenAPIRequest, RequestsOpenAPIResponse
from openapi_core.validation.response import openapi_response_validator

# Validate response against OpenAPI schema
spec = Spec.from_file_path("openapi.yaml")
openapi_request = RequestsOpenAPIRequest(request)
openapi_response = RequestsOpenAPIResponse(response)

result = openapi_response_validator.validate(spec, openapi_request, openapi_response)
if result.errors:
    for error in result.errors:
        logger.error(f"Schema validation error: {error}")
```

**Why**: Catches API contract violations automatically

#### 3. **Custom Schema Validators**
```python
from jsonschema import validate, ValidationError

# Validate response body against specific schema
user_schema = {
    "type": "object",
    "properties": {
        "id": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "created_at": {"type": "string", "format": "date-time"}
    },
    "required": ["id", "email"]
}

try:
    validate(instance=response_json, schema=user_schema)
except ValidationError as e:
    logger.error(f"Response doesn't match schema: {e.message}")
```

**Why**: Provides fine-grained validation for specific endpoints

### References
- jsonschema Documentation: https://python-jsonschema.readthedocs.io/
- openapi-spec-validator: https://github.com/p1c2u/openapi-spec-validator
- OpenAPI 3.1 Specification: https://spec.openapis.org/oas/v3.1.0

---

## 4. HTML Report Templating

### Decision: Use Jinja2 for HTML Report Generation

**Rationale**:
- Industry standard for Python templating
- Powerful template inheritance and macros
- Automatic HTML escaping (security)
- Excellent documentation and community support
- Already a dependency of FastAPI/Starlette

**Alternatives Considered**:
- **Mako**: Less popular, more complex syntax
- **Chameleon**: Slower, less widely used
- **String formatting**: Error-prone, no escaping, hard to maintain

### Report Templating Patterns

#### 1. **Template Structure**
```html
<!-- base.html - Base template with layout -->
<!DOCTYPE html>
<html>
<head>
    <title>{% block title %}QA Report{% endblock %}</title>
    <link rel="stylesheet" href="styles.css">
</head>
<body>
    <header>
        <h1>Automated QA Workflow Report</h1>
    </header>
    <main>
        {% block content %}{% endblock %}
    </main>
    <footer>
        Generated on {{ timestamp }}
    </footer>
</body>
</html>

<!-- report.html - Report template extending base -->
{% extends "base.html" %}

{% block content %}
<section class="summary">
    <h2>Summary</h2>
    <div class="stats">
        <div class="stat critical">
            <span class="count">{{ summary.critical_issues }}</span>
            <span class="label">Critical</span>
        </div>
        <!-- More stats... -->
    </div>
</section>

<section class="issues">
    <h2>Issues Detected</h2>
    <table>
        <thead>
            <tr>
                <th>Severity</th>
                <th>Type</th>
                <th>Location</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            {% for issue in issues %}
            <tr class="{{ issue.severity }}">
                <td>{{ issue.severity|upper }}</td>
                <td>{{ issue.type }}</td>
                <td><code>{{ issue.location }}</code></td>
                <td>{{ issue.description }}</td>
            </tr>
            {% endfor %}
        </tbody>
    </table>
</section>
{% endblock %}
```

#### 2. **Report Generation**
```python
from jinja2 import Environment, FileSystemLoader

# Setup Jinja2 environment
env = Environment(
    loader=FileSystemLoader("templates"),
    autoescape=True  # Prevent XSS
)

# Render report
template = env.get_template("report.html")
html = template.render(
    summary=workflow.summary,
    issues=workflow.issues,
    timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
)

# Write to file
with open("reports/dashboard.html", "w") as f:
    f.write(html)
```

#### 3. **Custom Filters**
```python
# Add custom filters for formatting
def format_duration(seconds):
    minutes = seconds // 60
    seconds = seconds % 60
    return f"{minutes}m {seconds}s"

env.filters["duration"] = format_duration

# Use in template: {{ workflow.duration|duration }}
```

#### 4. **Embedded Visualizations**
```html
<!-- Include Chart.js for visualizations -->
<canvas id="severityChart"></canvas>
<script>
    const ctx = document.getElementById('severityChart');
    new Chart(ctx, {
        type: 'doughnut',
        data: {
            labels: ['Critical', 'High', 'Medium', 'Low'],
            datasets: [{
                data: [
                    {{ summary.critical_issues }},
                    {{ summary.high_issues }},
                    {{ summary.medium_issues }},
                    {{ summary.low_issues }}
                ],
                backgroundColor: ['#dc3545', '#fd7e14', '#ffc107', '#28a745']
            }]
        }
    });
</script>
```

### References
- Jinja2 Documentation: https://jinja.palletsprojects.com/
- Template Designer Documentation: https://jinja.palletsprojects.com/en/3.1.x/templates/
- Chart.js: https://www.chartjs.org/

---

## 5. Automated Fix Patterns

### Decision: Use AST-based Transformations + External Tools

**Rationale**:
- **AST (Abstract Syntax Tree)**: Safe, preserves code structure, type-safe
- **External Tools**: Leverage existing linters/formatters (ESLint, Black, Ruff)
- **Safety First**: Only fix low-risk issues, backup files, validate after fix

**Automated Fix Categories**:

#### 1. **Safe Fixes (AST-based)**
```python
import ast
import astunparse  # or use ast.unparse in Python 3.9+

# Example: Remove unused imports
class UnusedImportRemover(ast.NodeTransformer):
    def __init__(self, used_names):
        self.used_names = used_names

    def visit_Import(self, node):
        # Only keep imports that are used
        node.names = [alias for alias in node.names
                      if alias.name in self.used_names]
        return node if node.names else None

# Parse, transform, unparse
tree = ast.parse(source_code)
transformer = UnusedImportRemover(used_names)
new_tree = transformer.visit(tree)
fixed_code = astunparse.unparse(new_tree)
```

#### 2. **Formatting Fixes (External Tools)**
```python
import subprocess

# Run Black formatter
subprocess.run(["black", "--line-length=100", file_path], check=True)

# Run ESLint with auto-fix
subprocess.run(["npx", "eslint", "--fix", file_path], check=True)

# Run Ruff linter with auto-fix
subprocess.run(["ruff", "check", "--fix", file_path], check=True)
```

#### 3. **Pattern-based Fixes (Regex)**
```python
import re

# Example: Add missing alt text to images
def fix_missing_alt_text(html_content):
    pattern = r'<img(?![^>]*\salt=)([^>]*)>'
    replacement = r'<img alt=""\\1>'
    return re.sub(pattern, replacement, html_content)

# Example: Fix console.log to use logger
def fix_console_log(js_content):
    pattern = r'console\.(log|error|warn)\('
    replacement = r'logger.\\1('
    return re.sub(pattern, replacement, js_content)
```

**Why**: Pattern matching for simple textual fixes, AST for code structure changes

### Safety Mechanisms

#### 1. **Backup Before Fix**
```python
import shutil
from pathlib import Path

def create_backup(file_path: Path) -> Path:
    backup_path = file_path.with_suffix(file_path.suffix + ".bak")
    shutil.copy2(file_path, backup_path)
    return backup_path

def restore_backup(backup_path: Path, original_path: Path):
    shutil.copy2(backup_path, original_path)
    backup_path.unlink()  # Delete backup
```

#### 2. **Syntax Validation**
```python
def validate_syntax(file_path: Path) -> bool:
    """Validate syntax after fix"""
    if file_path.suffix == ".py":
        try:
            with open(file_path) as f:
                ast.parse(f.read())
            return True
        except SyntaxError:
            return False

    elif file_path.suffix in [".js", ".ts", ".tsx"]:
        result = subprocess.run(
            ["npx", "eslint", "--parser-options=ecmaVersion:latest", str(file_path)],
            capture_output=True
        )
        return result.returncode == 0

    return True  # Unknown file type, assume valid
```

#### 3. **Test Verification**
```python
def run_related_tests(file_path: Path) -> bool:
    """Run tests related to the modified file"""
    # Find test file
    test_file = file_path.parent / f"test_{file_path.stem}.py"

    if not test_file.exists():
        return True  # No tests, assume pass

    # Run tests
    result = subprocess.run(
        ["pytest", str(test_file), "-v"],
        capture_output=True
    )
    return result.returncode == 0
```

### References
- Python AST Documentation: https://docs.python.org/3/library/ast.html
- Black Code Formatter: https://black.readthedocs.io/
- ESLint: https://eslint.org/docs/latest/use/getting-started
- Ruff Linter: https://docs.astral.sh/ruff/

---

## 6. Performance Optimization Patterns

### Decision: Parallel Execution + Caching + Resource Pooling

#### 1. **Parallel Test Execution**
```python
# Use asyncio.Semaphore to limit concurrent operations
async def run_tests_with_concurrency_limit(scenarios, max_concurrent=10):
    semaphore = asyncio.Semaphore(max_concurrent)

    async def run_with_limit(scenario):
        async with semaphore:
            return await run_test(scenario)

    return await asyncio.gather(*[run_with_limit(s) for s in scenarios])
```

**Why**: Prevents resource exhaustion while maximizing parallelism

#### 2. **Browser Context Reuse**
```python
# Reuse browser contexts for auth tests
async def run_auth_tests(scenarios):
    browser = await playwright.chromium.launch()

    # Create authenticated context once
    context = await browser.new_context()
    await authenticate(context)

    # Reuse for all auth tests
    results = []
    for scenario in scenarios:
        page = await context.new_page()
        result = await run_test_on_page(page, scenario)
        results.append(result)
        await page.close()

    await context.close()
    await browser.close()
    return results
```

**Why**: Authentication is expensive, reuse sessions across tests

#### 3. **Result Caching**
```python
from functools import lru_cache
import hashlib

# Cache API responses for repeated calls
@lru_cache(maxsize=1000)
async def fetch_openapi_spec(url: str):
    async with httpx.AsyncClient() as client:
        response = await client.get(url)
        return response.json()

# Cache issue analysis results
def cache_key_for_issue(issue: Issue) -> str:
    content = f"{issue.type}{issue.location}{issue.description}"
    return hashlib.md5(content.encode()).hexdigest()

analysis_cache = {}

def analyze_issue_cached(issue: Issue) -> dict:
    key = cache_key_for_issue(issue)
    if key not in analysis_cache:
        analysis_cache[key] = analyze_issue(issue)
    return analysis_cache[key]
```

**Why**: Avoids redundant work for identical issues

### References
- asyncio Documentation: https://docs.python.org/3/library/asyncio.html
- Python Performance Tips: https://wiki.python.org/moin/PythonSpeed/PerformanceTips

---

## 7. Error Handling Strategies

### Decision: Fail-Fast for Critical Errors, Continue for Non-Critical

#### 1. **Error Classification**
```python
class CriticalError(Exception):
    """Errors that should stop the workflow"""
    pass

class NonCriticalError(Exception):
    """Errors that should be logged but workflow continues"""
    pass

# Environment setup failures → Critical
try:
    browser = await playwright.chromium.launch()
except Exception as e:
    raise CriticalError(f"Browser launch failed: {e}")

# Individual test failures → Non-critical
try:
    await page.goto(url)
except Exception as e:
    log_error(NonCriticalError(f"Test failed: {e}"))
    continue  # Continue with next test
```

#### 2. **Retry with Exponential Backoff**
```python
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((httpx.TimeoutException, httpx.ConnectError))
)
async def call_api_with_retry(client, url):
    response = await client.get(url, timeout=10.0)
    response.raise_for_status()
    return response
```

#### 3. **Circuit Breaker Pattern**
```python
class CircuitBreaker:
    def __init__(self, failure_threshold=5, timeout=60):
        self.failure_count = 0
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.last_failure_time = None
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN

    async def call(self, func, *args, **kwargs):
        if self.state == "OPEN":
            if time.time() - self.last_failure_time > self.timeout:
                self.state = "HALF_OPEN"
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = await func(*args, **kwargs)
            if self.state == "HALF_OPEN":
                self.state = "CLOSED"
                self.failure_count = 0
            return result
        except Exception as e:
            self.failure_count += 1
            self.last_failure_time = time.time()
            if self.failure_count >= self.failure_threshold:
                self.state = "OPEN"
            raise e
```

**Why**: Prevents cascading failures when external services are down

### References
- Tenacity Documentation: https://tenacity.readthedocs.io/
- Circuit Breaker Pattern: https://martinfowler.com/bliki/CircuitBreaker.html

---

## Summary of Technology Decisions

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Browser Automation | Playwright | Modern, fast, reliable, multi-browser, auto-waiting |
| API Testing | httpx AsyncClient | Async support, HTTP/2, performance, compatibility |
| Schema Validation | jsonschema + openapi-spec-validator | Standard, mature, detailed error messages |
| HTML Reports | Jinja2 | Industry standard, secure, powerful templating |
| Automated Fixes | AST + External Tools | Safe transformations, leverage existing tools |
| Performance | asyncio + Caching + Pooling | Maximize parallelism, avoid redundant work |
| Error Handling | Fail-fast + Retry + Circuit Breaker | Balance robustness and failure recovery |

**All decisions documented and ready for implementation in Phase 1 (Design)**.
