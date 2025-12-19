"""Severity analyzer for assigning severity levels to issues."""

import logging
from typing import Any

from qa_automation.models.issue import Issue, SeverityLevel

logger = logging.getLogger(__name__)


class SeverityAnalyzer:
    """
    Analyzes issues and assigns appropriate severity levels.

    Severity levels:
    - CRITICAL: Blocks deployment (auth bypass, SQL injection, data corruption, 500 errors)
    - HIGH: Significant impact (404, 403, broken layout, JS errors, failed requests)
    - MEDIUM: Moderate impact (slow response, missing alt text, deprecated API)
    - LOW: Minor impact (cosmetic issues, warnings)
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize severity analyzer.

        Args:
            config: Configuration dictionary
        """
        self.config = config

        # Critical patterns
        self.critical_patterns = [
            "authentication bypass",
            "sql injection",
            "xss vulnerability",
            "cross-site scripting",
            "data corruption",
            "500 internal server error",
            "csrf vulnerability",
            "remote code execution",
            "privilege escalation",
        ]

        # High severity patterns
        self.high_patterns = [
            "404 not found",
            "403 forbidden",
            "401 unauthorized",
            "broken layout",
            "javascript error",
            "console error",
            "failed to load",
            "timeout",
            "connection refused",
            "missing element",
        ]

        # Medium severity patterns
        self.medium_patterns = [
            "slow response",
            "missing alt text",
            "poor contrast",
            "deprecated api",
            "warning",
            "accessibility",
            "performance",
        ]

    def assign_severity(self, issue: Issue) -> SeverityLevel:
        """
        Assign severity level to an issue.

        Args:
            issue: Issue to analyze

        Returns:
            Assigned severity level
        """
        description_lower = issue.description.lower()

        # Check critical patterns
        if any(pattern in description_lower for pattern in self.critical_patterns):
            logger.debug(f"Issue {issue.id} assigned CRITICAL severity")
            return SeverityLevel.CRITICAL

        # Check high patterns
        if any(pattern in description_lower for pattern in self.high_patterns):
            logger.debug(f"Issue {issue.id} assigned HIGH severity")
            return SeverityLevel.HIGH

        # Check medium patterns
        if any(pattern in description_lower for pattern in self.medium_patterns):
            logger.debug(f"Issue {issue.id} assigned MEDIUM severity")
            return SeverityLevel.MEDIUM

        # Default to low
        logger.debug(f"Issue {issue.id} assigned LOW severity")
        return SeverityLevel.LOW

    def analyze_issues(self, issues: list[Issue]) -> list[Issue]:
        """
        Analyze and assign severity to all issues.

        Args:
            issues: List of issues to analyze

        Returns:
            Issues with severity assigned
        """
        logger.info(f"Assigning severity to {len(issues)} issues")

        for issue in issues:
            if issue.severity is None:
                issue.severity = self.assign_severity(issue)

        # Log severity distribution
        severity_counts = {
            SeverityLevel.CRITICAL: sum(1 for i in issues if i.severity == SeverityLevel.CRITICAL),
            SeverityLevel.HIGH: sum(1 for i in issues if i.severity == SeverityLevel.HIGH),
            SeverityLevel.MEDIUM: sum(1 for i in issues if i.severity == SeverityLevel.MEDIUM),
            SeverityLevel.LOW: sum(1 for i in issues if i.severity == SeverityLevel.LOW),
        }

        logger.info(
            f"Severity distribution: Critical={severity_counts[SeverityLevel.CRITICAL]}, "
            f"High={severity_counts[SeverityLevel.HIGH]}, "
            f"Medium={severity_counts[SeverityLevel.MEDIUM]}, "
            f"Low={severity_counts[SeverityLevel.LOW]}"
        )

        return issues
