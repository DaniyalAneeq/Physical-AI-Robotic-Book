"""Performance detector for tracking response times and metrics."""

import logging
from typing import Any

from qa_automation.detectors.base import BaseDetector
from qa_automation.models.issue import Issue, IssueType, ResolutionStatus
from qa_automation.models.test_result import TestResult

logger = logging.getLogger(__name__)


class PerformanceDetector(BaseDetector):
    """
    Performance detector for analyzing metrics from other detectors.

    Analyzes:
    - Page load times
    - API response times
    - Bundle sizes
    - Resource counts
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize performance detector.

        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        self.performance_config = config.get("performance", {})

    async def detect(self) -> tuple[list[TestResult], list[Issue]]:
        """
        Performance detector doesn't run independently.
        It analyzes results from frontend and backend detectors.

        Returns:
            Empty tuple (performance issues are detected inline by other detectors)
        """
        return [], []

    def analyze_test_results(self, test_results: list[TestResult]) -> list[Issue]:
        """
        Analyze test results and detect performance issues.

        Args:
            test_results: List of test results from frontend/backend detectors

        Returns:
            List of detected performance issues
        """
        performance_issues = []

        for result in test_results:
            # Check page load time (frontend)
            if "page_load_ms" in result.performance_metrics:
                max_page_load = self.performance_config.get("max_page_load_ms", 3000)
                page_load = result.performance_metrics["page_load_ms"]

                if page_load > max_page_load:
                    issue = Issue(
                        type=IssueType.PERFORMANCE,
                        location=f"Scenario: {result.scenario_id}",
                        description=f"Slow page load: {page_load}ms (threshold: {max_page_load}ms)",
                        captured_context={
                            "page_load_ms": page_load,
                            "threshold_ms": max_page_load,
                            "scenario_id": result.scenario_id,
                        },
                        resolution_status=ResolutionStatus.PENDING,
                    )
                    performance_issues.append(issue)

            # Check API response time (backend)
            if "response_time_ms" in result.performance_metrics:
                max_api_response = self.performance_config.get(
                    "max_api_response_ms", 1000
                )
                response_time = result.performance_metrics["response_time_ms"]

                if response_time > max_api_response:
                    issue = Issue(
                        type=IssueType.PERFORMANCE,
                        location=f"Scenario: {result.scenario_id}",
                        description=f"Slow API response: {response_time}ms (threshold: {max_api_response}ms)",
                        captured_context={
                            "response_time_ms": response_time,
                            "threshold_ms": max_api_response,
                            "scenario_id": result.scenario_id,
                        },
                        resolution_status=ResolutionStatus.PENDING,
                    )
                    performance_issues.append(issue)

        if performance_issues:
            logger.info(f"Detected {len(performance_issues)} performance issues")

        return performance_issues
