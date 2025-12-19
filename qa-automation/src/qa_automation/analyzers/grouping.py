"""Issue grouping analyzer for linking related issues."""

import logging
from difflib import SequenceMatcher
from typing import Any

from qa_automation.models.issue import Issue

logger = logging.getLogger(__name__)


class GroupingAnalyzer:
    """
    Groups related issues together.

    Grouping strategies:
    1. Same location (URL/endpoint)
    2. Cause-effect relationships (via root_cause)
    3. Similar descriptions (string similarity >0.8)
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize grouping analyzer.

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.similarity_threshold = 0.8

    def group_related_issues(self, issues: list[Issue]) -> list[Issue]:
        """
        Group related issues.

        Args:
            issues: List of issues to group

        Returns:
            Issues with related_issues populated
        """
        logger.info(f"Grouping {len(issues)} related issues")

        for issue in issues:
            related = set()

            # Find issues at same location
            location_matches = [
                i for i in issues if i.location == issue.location and i.id != issue.id
            ]
            related.update(i.id for i in location_matches)

            # Find cause-effect relationships
            related_by_cause = [
                i
                for i in issues
                if (issue.id in (i.root_cause or ""))
                or (i.id in (issue.root_cause or ""))
            ]
            related.update(i.id for i in related_by_cause)

            # Find similar descriptions (fuzzy matching)
            similar_desc = [
                i
                for i in issues
                if self._calculate_similarity(issue.description, i.description)
                > self.similarity_threshold
                and i.id != issue.id
            ]
            related.update(i.id for i in similar_desc)

            # Update issue with related issues
            issue.related_issues = list(related)

        # Log grouping statistics
        total_relationships = sum(len(i.related_issues) for i in issues)
        avg_relationships = (
            total_relationships / len(issues) if issues else 0
        )

        logger.info(
            f"Grouping complete: {total_relationships} relationships, "
            f"avg {avg_relationships:.1f} per issue"
        )

        return issues

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
        Analyze and group related issues.

        Args:
            issues: List of issues to analyze

        Returns:
            Issues with relationships identified
        """
        return self.group_related_issues(issues)
