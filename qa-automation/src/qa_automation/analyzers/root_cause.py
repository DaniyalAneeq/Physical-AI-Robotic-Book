"""Root cause analyzer for identifying issue dependencies."""

import logging
from difflib import SequenceMatcher
from typing import Any

from qa_automation.models.issue import Issue, IssueType

logger = logging.getLogger(__name__)


class RootCauseAnalyzer:
    """
    Identifies root causes by analyzing issue relationships.

    Strategies:
    1. Timing analysis: Earlier issues may cause later ones
    2. Dependency detection: Backend errors causing frontend failures
    3. Pattern detection: Similar issues across multiple locations
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize root cause analyzer.

        Args:
            config: Configuration dictionary
        """
        self.config = config

    def identify_root_cause(self, issue: Issue, all_issues: list[Issue]) -> str:
        """
        Identify root cause for an issue.

        Args:
            issue: Issue to analyze
            all_issues: All detected issues for context

        Returns:
            Root cause description
        """
        # Check if this is a symptom of a backend issue
        if issue.type == IssueType.FRONTEND:
            backend_cause = self._check_backend_dependency(issue, all_issues)
            if backend_cause:
                return backend_cause

        # Check for pattern across multiple issues
        similar_issues = self._find_similar_issues(issue, all_issues)
        if len(similar_issues) > 3:
            return (
                f"Systemic issue: {len(similar_issues)} similar occurrences detected. "
                f"Related issues: {', '.join([i.id for i in similar_issues[:3]])}"
            )

        # Default: issue is its own root cause
        return "Direct issue - no upstream dependency detected"

    def _check_backend_dependency(
        self, frontend_issue: Issue, all_issues: list[Issue]
    ) -> str | None:
        """
        Check if frontend issue is caused by backend error.

        Args:
            frontend_issue: Frontend issue to analyze
            all_issues: All issues

        Returns:
            Root cause description if backend dependency found, None otherwise
        """
        # Look for backend errors with earlier timestamps
        for other in all_issues:
            if (
                other.type == IssueType.BACKEND
                and other.detection_timestamp < frontend_issue.detection_timestamp
                and self._are_related(frontend_issue, other)
            ):
                return (
                    f"Caused by backend issue: {other.id} - {other.description[:50]}..."
                )

        return None

    def _are_related(self, issue1: Issue, issue2: Issue) -> bool:
        """
        Check if two issues are related.

        Args:
            issue1: First issue
            issue2: Second issue

        Returns:
            True if issues are related
        """
        # Check location similarity
        if issue1.location in issue2.description or issue2.location in issue1.description:
            return True

        # Check description similarity
        similarity = self._calculate_similarity(issue1.description, issue2.description)
        return similarity > 0.5

    def _find_similar_issues(self, issue: Issue, all_issues: list[Issue]) -> list[Issue]:
        """
        Find issues similar to the given issue.

        Args:
            issue: Issue to compare
            all_issues: All issues

        Returns:
            List of similar issues
        """
        similar = []

        for other in all_issues:
            if other.id == issue.id:
                continue

            # Same type
            if other.type != issue.type:
                continue

            # Similar description
            similarity = self._calculate_similarity(issue.description, other.description)
            if similarity > 0.8:
                similar.append(other)

        return similar

    def _calculate_similarity(self, str1: str, str2: str) -> float:
        """
        Calculate similarity ratio between two strings.

        Args:
            str1: First string
            str2: Second string

        Returns:
            Similarity ratio (0.0 to 1.0)
        """
        return SequenceMatcher(None, str1.lower(), str2.lower()).ratio()

    def analyze_issues(self, issues: list[Issue]) -> list[Issue]:
        """
        Analyze and identify root causes for all issues.

        Args:
            issues: List of issues to analyze

        Returns:
            Issues with root causes identified
        """
        logger.info(f"Identifying root causes for {len(issues)} issues")

        for issue in issues:
            if not issue.root_cause:
                issue.root_cause = self.identify_root_cause(issue, issues)

        # Count root causes identified
        identified = sum(
            1
            for i in issues
            if i.root_cause and "no upstream dependency" not in i.root_cause
        )
        percentage = (identified / len(issues) * 100) if issues else 0

        logger.info(f"Root causes identified for {identified}/{len(issues)} issues ({percentage:.1f}%)")

        return issues
