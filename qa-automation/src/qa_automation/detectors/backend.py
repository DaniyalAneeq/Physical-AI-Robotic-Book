"""Backend detector using httpx for API testing."""

import logging
from datetime import datetime
from typing import Any

import httpx
import yaml

from qa_automation.detectors.base import BaseDetector
from qa_automation.models.issue import Issue, IssueType, ResolutionStatus
from qa_automation.models.test_result import TestResult, TestStatus
from qa_automation.models.test_scenario import TestScenario
from qa_automation.services.auth_handler import AuthHandler, AuthenticationError

logger = logging.getLogger(__name__)


class BackendDetector(BaseDetector):
    """
    Backend detector using httpx for API testing.

    Detects:
    - Status code errors (4xx, 5xx)
    - Response schema violations
    - Timeout issues
    - Slow responses (>1s)
    - Authentication failures
    """

    def __init__(self, config: dict[str, Any], workflow_id: str):
        """
        Initialize backend detector.

        Args:
            config: Configuration dictionary
            workflow_id: Current workflow execution ID
        """
        super().__init__(config)
        self.workflow_id = workflow_id
        self.auth_handler = AuthHandler(config)

        # Load test scenarios
        scenarios_file = config.get("scenarios", {}).get("backend")
        self.scenarios = self._load_scenarios(scenarios_file)

    def _load_scenarios(self, scenarios_file: str | None) -> list[TestScenario]:
        """
        Load backend test scenarios from YAML file.

        Args:
            scenarios_file: Path to scenarios YAML file

        Returns:
            List of TestScenario objects
        """
        if not scenarios_file:
            logger.warning("No backend scenarios file specified")
            return []

        try:
            with open(scenarios_file) as f:
                data = yaml.safe_load(f)

            scenarios = []
            for scenario_data in data.get("scenarios", []):
                scenario = TestScenario(**scenario_data)
                if scenario.enabled:
                    scenarios.append(scenario)

            logger.info(f"Loaded {len(scenarios)} backend test scenarios")
            return scenarios

        except Exception as e:
            logger.error(f"Failed to load backend scenarios: {e}")
            return []

    async def detect(self) -> tuple[list[TestResult], list[Issue]]:
        """
        Execute backend detection and return results.

        Returns:
            Tuple of (test_results, detected_issues)
        """
        test_results = []

        # Create HTTP client
        timeout = self.config.get("timeout", 30000) / 1000  # Convert ms to seconds
        async with httpx.AsyncClient(
            timeout=httpx.Timeout(timeout),
            follow_redirects=True,
        ) as client:
            # Run each scenario
            for scenario in self.scenarios:
                result = await self._run_scenario(client, scenario)
                test_results.append(result)

        return test_results, self.detected_issues

    async def _run_scenario(
        self, client: httpx.AsyncClient, scenario: TestScenario
    ) -> TestResult:
        """
        Run a single test scenario.

        Args:
            client: httpx AsyncClient
            scenario: Test scenario to run

        Returns:
            TestResult for this scenario
        """
        start_time = datetime.utcnow()
        detected_issue_ids = []

        try:
            # Authenticate if required
            if scenario.auth_required:
                try:
                    await self.auth_handler.authenticate_api(
                        client, scenario.auth_method.value
                    )
                except AuthenticationError as e:
                    logger.error(f"Authentication failed for {scenario.id}: {e}")
                    return self._create_error_result(
                        scenario, start_time, f"Authentication failed: {e}"
                    )

            # Execute request
            # Construct full URL (prepend backend_url if URL is relative)
            full_url = scenario.url
            if scenario.url and not scenario.url.startswith(("http://", "https://")):
                backend_url = self.config.get("backend_url", "")
                full_url = f"{backend_url.rstrip('/')}/{scenario.url.lstrip('/')}"

            logger.info(
                f"Running scenario: {scenario.name} ({scenario.method} {full_url})"
            )

            response = await client.request(
                method=scenario.method,
                url=full_url,
                headers=scenario.headers,
                json=scenario.body,
            )

            # Calculate metrics
            duration = int((datetime.utcnow() - start_time).total_seconds() * 1000)

            # Detect issues
            detected_issue_ids = await self._detect_issues(
                response, scenario, duration
            )

            # Determine test status
            expected_status = scenario.expected_status if hasattr(scenario, 'expected_status') else None
            if expected_status and response.status_code != expected_status:
                status = TestStatus.FAILED
            else:
                status = TestStatus.PASSED

            # Create test result
            return TestResult(
                scenario_id=scenario.id,
                workflow_execution_id=self.workflow_id,
                execution_timestamp=start_time,
                status=status,
                duration_ms=duration,
                detected_issues=detected_issue_ids,
                performance_metrics={
                    "response_time_ms": duration,
                    "status_code": response.status_code,
                    "response_size_bytes": len(response.content),
                },
                captured_artifacts={
                    "request": {
                        "method": scenario.method,
                        "url": scenario.url,
                        "headers": dict(scenario.headers),
                        "body": scenario.body,
                    },
                    "response": {
                        "status": response.status_code,
                        "headers": dict(response.headers),
                        "body": response.text[:1000],  # First 1000 chars
                    },
                },
            )

        except httpx.TimeoutException as e:
            logger.error(f"Scenario {scenario.id} timed out: {e}")
            issue = self._create_timeout_issue(scenario)
            self.add_issue(issue)
            return self._create_timeout_result(scenario, start_time)

        except httpx.HTTPError as e:
            logger.error(f"Scenario {scenario.id} failed: {e}")
            return self._create_error_result(scenario, start_time, str(e))

    async def _detect_issues(
        self, response: httpx.Response, scenario: TestScenario, duration_ms: int
    ) -> list[str]:
        """
        Detect issues in the API response.

        Args:
            response: httpx Response
            scenario: Current scenario
            duration_ms: Response time in milliseconds

        Returns:
            List of detected issue IDs
        """
        detected_issue_ids = []

        # Detect status code errors
        if response.status_code >= 500:
            issue = self._create_server_error_issue(response, scenario)
            self.add_issue(issue)
            detected_issue_ids.append(issue.id)
        elif response.status_code >= 400:
            issue = self._create_client_error_issue(response, scenario)
            self.add_issue(issue)
            detected_issue_ids.append(issue.id)

        # Detect slow responses
        performance_threshold = scenario.performance_thresholds.get(
            "max_response_time_ms", 1000
        )
        if duration_ms > performance_threshold:
            issue = self._create_slow_response_issue(
                response, scenario, duration_ms, performance_threshold
            )
            self.add_issue(issue)
            detected_issue_ids.append(issue.id)

        return detected_issue_ids

    def _create_server_error_issue(
        self, response: httpx.Response, scenario: TestScenario
    ) -> Issue:
        """Create issue for 5xx server error."""
        return Issue(
            type=IssueType.BACKEND,
            location=f"{scenario.method} {scenario.url}",
            description=f"500 internal server error: {response.status_code} {response.reason_phrase}",
            captured_context={
                "status_code": response.status_code,
                "method": scenario.method,
                "url": scenario.url,
                "response_body": response.text[:500],
                "headers": dict(response.headers),
            },
            resolution_status=ResolutionStatus.PENDING,
        )

    def _create_client_error_issue(
        self, response: httpx.Response, scenario: TestScenario
    ) -> Issue:
        """Create issue for 4xx client error."""
        return Issue(
            type=IssueType.BACKEND,
            location=f"{scenario.method} {scenario.url}",
            description=f"Client error: {response.status_code} {response.reason_phrase}",
            captured_context={
                "status_code": response.status_code,
                "method": scenario.method,
                "url": scenario.url,
                "response_body": response.text[:500],
            },
            resolution_status=ResolutionStatus.PENDING,
        )

    def _create_slow_response_issue(
        self,
        response: httpx.Response,
        scenario: TestScenario,
        duration_ms: int,
        threshold_ms: int,
    ) -> Issue:
        """Create issue for slow response."""
        return Issue(
            type=IssueType.PERFORMANCE,
            location=f"{scenario.method} {scenario.url}",
            description=f"Slow API response: {duration_ms}ms (threshold: {threshold_ms}ms)",
            captured_context={
                "response_time_ms": duration_ms,
                "threshold_ms": threshold_ms,
                "method": scenario.method,
                "url": scenario.url,
                "status_code": response.status_code,
            },
            resolution_status=ResolutionStatus.PENDING,
        )

    def _create_timeout_issue(self, scenario: TestScenario) -> Issue:
        """Create issue for timeout."""
        return Issue(
            type=IssueType.PERFORMANCE,
            location=f"{scenario.method} {scenario.url}",
            description=f"API request timeout exceeded",
            captured_context={
                "method": scenario.method,
                "url": scenario.url,
                "timeout": self.config.get("timeout", 30000),
            },
            resolution_status=ResolutionStatus.PENDING,
        )

    def _create_error_result(
        self, scenario: TestScenario, start_time: datetime, error_message: str
    ) -> TestResult:
        """Create error test result."""
        duration = int((datetime.utcnow() - start_time).total_seconds() * 1000)

        return TestResult(
            scenario_id=scenario.id,
            workflow_execution_id=self.workflow_id,
            execution_timestamp=start_time,
            status=TestStatus.ERROR,
            duration_ms=duration,
            error_message=error_message,
            detected_issues=[],
            performance_metrics={},
            captured_artifacts={},
        )

    def _create_timeout_result(
        self, scenario: TestScenario, start_time: datetime
    ) -> TestResult:
        """Create timeout test result."""
        duration = int((datetime.utcnow() - start_time).total_seconds() * 1000)

        return TestResult(
            scenario_id=scenario.id,
            workflow_execution_id=self.workflow_id,
            execution_timestamp=start_time,
            status=TestStatus.TIMEOUT,
            duration_ms=duration,
            error_message="Request timeout exceeded",
            detected_issues=[],
            performance_metrics={},
            captured_artifacts={},
        )
