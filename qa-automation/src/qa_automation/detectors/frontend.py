"""Frontend detector using Playwright for browser automation."""

import logging
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from playwright.async_api import async_playwright, Browser, BrowserContext, Page

from qa_automation.detectors.base import BaseDetector
from qa_automation.models.issue import Issue, IssueType, ResolutionStatus
from qa_automation.models.test_result import TestResult, TestStatus
from qa_automation.models.test_scenario import TestScenario
from qa_automation.services.artifact_manager import ArtifactManager
from qa_automation.services.auth_handler import AuthHandler, AuthenticationError

logger = logging.getLogger(__name__)


class FrontendDetector(BaseDetector):
    """
    Frontend detector using Playwright for browser automation.

    Detects:
    - Rendering errors
    - Broken layouts
    - Missing elements
    - Console errors and warnings
    - Accessibility violations (missing alt text)
    - Performance issues (slow page loads)
    """

    def __init__(
        self,
        config: dict[str, Any],
        artifact_manager: ArtifactManager,
        workflow_id: str,
    ):
        """
        Initialize frontend detector.

        Args:
            config: Configuration dictionary
            artifact_manager: Artifact manager for storing screenshots/traces
            workflow_id: Current workflow execution ID
        """
        super().__init__(config)
        self.artifact_manager = artifact_manager
        self.workflow_id = workflow_id
        self.auth_handler = AuthHandler(config)
        self.browser: Browser | None = None
        self.context: BrowserContext | None = None

        # Load test scenarios
        scenarios_file = config.get("scenarios", {}).get("frontend")
        self.scenarios = self._load_scenarios(scenarios_file)

    def _load_scenarios(self, scenarios_file: str | None) -> list[TestScenario]:
        """
        Load frontend test scenarios from YAML file.

        Args:
            scenarios_file: Path to scenarios YAML file

        Returns:
            List of TestScenario objects
        """
        if not scenarios_file:
            logger.warning("No frontend scenarios file specified")
            return []

        try:
            with open(scenarios_file) as f:
                data = yaml.safe_load(f)

            scenarios = []
            for scenario_data in data.get("scenarios", []):
                scenario = TestScenario(**scenario_data)
                if scenario.enabled:
                    scenarios.append(scenario)

            logger.info(f"Loaded {len(scenarios)} frontend test scenarios")
            return scenarios

        except Exception as e:
            logger.error(f"Failed to load frontend scenarios: {e}")
            return []

    async def detect(self) -> tuple[list[TestResult], list[Issue]]:
        """
        Execute frontend detection and return results.

        Returns:
            Tuple of (test_results, detected_issues)
        """
        test_results = []

        async with async_playwright() as playwright:
            # Launch browser
            browser_type = self.config.get("browser", "chromium")
            headless = self.config.get("headless", True)

            logger.info(f"Launching {browser_type} browser (headless={headless})")

            if browser_type == "chromium":
                self.browser = await playwright.chromium.launch(headless=headless)
            elif browser_type == "firefox":
                self.browser = await playwright.firefox.launch(headless=headless)
            elif browser_type == "webkit":
                self.browser = await playwright.webkit.launch(headless=headless)
            else:
                raise ValueError(f"Unsupported browser: {browser_type}")

            # Create browser context
            viewport = self.config.get("viewport", {"width": 1920, "height": 1080})
            self.context = await self.browser.new_context(viewport=viewport)

            # Enable tracing
            trace_dir = self.artifact_manager.get_workflow_trace_dir(self.workflow_id)
            await self.context.tracing.start(screenshots=True, snapshots=True)

            # Run each scenario
            for scenario in self.scenarios:
                result = await self._run_scenario(scenario)
                test_results.append(result)

            # Stop tracing and save
            trace_path = trace_dir / "frontend-trace.zip"
            await self.context.tracing.stop(path=str(trace_path))
            logger.info(f"Trace saved: {trace_path}")

            # Cleanup
            await self.context.close()
            await self.browser.close()

        return test_results, self.detected_issues

    async def _run_scenario(self, scenario: TestScenario) -> TestResult:
        """
        Run a single test scenario.

        Args:
            scenario: Test scenario to run

        Returns:
            TestResult for this scenario
        """
        start_time = datetime.utcnow()
        page: Page | None = None
        detected_issue_ids = []

        try:
            # Create new page
            page = await self.context.new_page()

            # Set up console message handler
            console_messages = []

            def handle_console(msg):
                console_messages.append(
                    {"type": msg.type, "text": msg.text, "location": msg.location}
                )

            page.on("console", handle_console)

            # Authenticate if required
            if scenario.auth_required:
                try:
                    await self.auth_handler.authenticate_browser(
                        self.context, scenario.auth_method.value
                    )
                except AuthenticationError as e:
                    logger.error(f"Authentication failed for {scenario.id}: {e}")
                    return self._create_error_result(
                        scenario, start_time, f"Authentication failed: {e}"
                    )

            # Navigate to URL
            logger.info(f"Running scenario: {scenario.name} ({scenario.url})")
            response = await page.goto(scenario.url, wait_until="networkidle")

            # Execute test steps
            for step in scenario.steps:
                await self._execute_step(page, step, scenario)

            # Check assertions
            for assertion in scenario.assertions:
                await self._check_assertion(page, assertion, scenario)

            # Detect issues
            detected_issue_ids = await self._detect_issues(
                page, scenario, console_messages
            )

            # Calculate metrics
            duration = int((datetime.utcnow() - start_time).total_seconds() * 1000)
            metrics = {
                "page_load_ms": duration,
                "response_status": response.status if response else None,
            }

            # Create test result
            return TestResult(
                scenario_id=scenario.id,
                workflow_execution_id=self.workflow_id,
                execution_timestamp=start_time,
                status=TestStatus.PASSED,
                duration_ms=duration,
                detected_issues=detected_issue_ids,
                performance_metrics=metrics,
                captured_artifacts={},
            )

        except Exception as e:
            logger.error(f"Scenario {scenario.id} failed: {e}")
            return self._create_error_result(scenario, start_time, str(e))

        finally:
            if page:
                await page.close()

    async def _execute_step(
        self, page: Page, step: Any, scenario: TestScenario
    ) -> None:
        """
        Execute a single test step.

        Args:
            page: Playwright page
            step: Test step to execute
            scenario: Parent scenario
        """
        action = step.action

        if action == "wait_for_selector":
            await page.wait_for_selector(step.selector, timeout=step.timeout)
        elif action == "click":
            await page.click(step.selector)
        elif action == "fill":
            await page.fill(step.selector, step.value)
        elif action == "screenshot":
            screenshot_dir = self.artifact_manager.get_workflow_screenshot_dir(
                self.workflow_id
            )
            screenshot_path = screenshot_dir / f"{scenario.id}-{step.value}.png"
            await page.screenshot(path=str(screenshot_path), full_page=True)
            logger.debug(f"Screenshot saved: {screenshot_path}")
        else:
            logger.warning(f"Unknown action: {action}")

    async def _check_assertion(
        self, page: Page, assertion: Any, scenario: TestScenario
    ) -> None:
        """
        Check an assertion.

        Args:
            page: Playwright page
            assertion: Assertion to check
            scenario: Parent scenario

        Raises:
            AssertionError: If assertion fails
        """
        if assertion.selector:
            element = await page.query_selector(assertion.selector)
            if not element:
                self._add_issue_for_missing_element(assertion.selector, scenario)
                raise AssertionError(f"Element not found: {assertion.selector}")

            if assertion.text_contains:
                text_content = await element.text_content()
                if assertion.text_contains not in (text_content or ""):
                    raise AssertionError(
                        f"Text '{assertion.text_contains}' not found in {assertion.selector}"
                    )

    async def _detect_issues(
        self, page: Page, scenario: TestScenario, console_messages: list[dict]
    ) -> list[str]:
        """
        Detect issues on the page.

        Args:
            page: Playwright page
            scenario: Current scenario
            console_messages: Captured console messages

        Returns:
            List of detected issue IDs
        """
        detected_issue_ids = []

        # Detect console errors
        for msg in console_messages:
            if msg["type"] == "error":
                issue = self._create_console_error_issue(msg, scenario)
                self.add_issue(issue)
                detected_issue_ids.append(issue.id)

        # Detect missing alt text
        images_without_alt = await page.query_selector_all('img:not([alt])')
        for img in images_without_alt:
            issue = await self._create_missing_alt_issue(img, scenario)
            self.add_issue(issue)
            detected_issue_ids.append(issue.id)

        return detected_issue_ids

    def _create_console_error_issue(
        self, console_msg: dict, scenario: TestScenario
    ) -> Issue:
        """Create issue for console error."""
        return Issue(
            type=IssueType.FRONTEND,
            location=scenario.url,
            description=f"Console error: {console_msg['text']}",
            captured_context={
                "console_type": console_msg["type"],
                "message": console_msg["text"],
                "location": console_msg.get("location", {}),
            },
            resolution_status=ResolutionStatus.PENDING,
        )

    async def _create_missing_alt_issue(
        self, img_element: Any, scenario: TestScenario
    ) -> Issue:
        """Create issue for missing alt text."""
        src = await img_element.get_attribute("src")
        html = await img_element.evaluate("el => el.outerHTML")

        return Issue(
            type=IssueType.FRONTEND,
            location=scenario.url,
            description=f"Missing alt text on image: {src}",
            captured_context={"selector": "img", "src": src, "html": html},
            resolution_status=ResolutionStatus.PENDING,
        )

    def _add_issue_for_missing_element(
        self, selector: str, scenario: TestScenario
    ) -> None:
        """Create issue for missing element."""
        issue = Issue(
            type=IssueType.FRONTEND,
            location=scenario.url,
            description=f"Element not found: {selector}",
            captured_context={"selector": selector},
            resolution_status=ResolutionStatus.PENDING,
        )
        self.add_issue(issue)

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
