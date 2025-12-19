"""Base detector interface."""

from abc import ABC, abstractmethod
from typing import Any

from qa_automation.models.issue import Issue
from qa_automation.models.test_result import TestResult


class BaseDetector(ABC):
    """
    Abstract base class for all detectors.

    Detectors are responsible for:
    1. Running test scenarios
    2. Detecting issues during test execution
    3. Capturing artifacts (screenshots, traces, logs)
    4. Collecting performance metrics
    5. Returning structured test results and detected issues
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize detector with configuration.

        Args:
            config: Configuration dictionary containing detector-specific settings
        """
        self.config = config
        self.detected_issues: list[Issue] = []

    @abstractmethod
    async def detect(self) -> tuple[list[TestResult], list[Issue]]:
        """
        Execute detection and return results.

        Returns:
            Tuple of (test_results, detected_issues)
        """
        pass

    def add_issue(self, issue: Issue) -> None:
        """
        Add a detected issue to the collection.

        Args:
            issue: Issue to add
        """
        self.detected_issues.append(issue)

    def clear_issues(self) -> None:
        """Clear all detected issues."""
        self.detected_issues = []

    async def cleanup(self) -> None:
        """
        Cleanup resources after detection completes.

        Override this method to implement cleanup logic
        (e.g., closing browser, HTTP client).
        """
        pass
