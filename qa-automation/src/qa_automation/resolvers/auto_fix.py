"""Auto-fix resolver for applying safe automated fixes."""

import logging
import re
import subprocess
from pathlib import Path
from typing import Any

from qa_automation.models.issue import Issue, ResolutionStatus
from qa_automation.models.resolution import (
    ResolutionAction,
    ActionType,
    VerificationStatus,
    ResolutionOutcome,
)
from qa_automation.resolvers.validator import FixValidator, ValidationError
from qa_automation.services.artifact_manager import ArtifactManager

logger = logging.getLogger(__name__)


# Safe auto-fix patterns
SAFE_AUTO_FIXES = {
    "missing_alt_text": {
        "pattern": r'<img(?![^>]*\salt=)([^>]*)>',
        "description": "Add empty alt attribute to images",
        "safety": "Low risk - adds accessibility without changing functionality",
    },
    "eslint_formatting": {
        "pattern": "eslint",
        "description": "Run ESLint with --fix flag",
        "safety": "Low risk - formatting only, no logic changes",
    },
    "unused_imports": {
        "pattern": "unused import",
        "description": "Remove unused imports",
        "safety": "Low risk - removes dead code",
    },
}

# Unsafe patterns that should NEVER be auto-fixed
UNSAFE_PATTERNS = [
    "authentication bypass",
    "sql injection",
    "logic error",
    "api contract violation",
    "database migration required",
    "xss vulnerability",
]


class AutoFixResolver:
    """
    Applies safe automated fixes to issues.

    Safety guards:
    1. Only fix patterns in SAFE_AUTO_FIXES
    2. Create backup before fix
    3. Validate syntax after fix
    4. Run tests if available
    5. Rollback on failure
    """

    def __init__(
        self,
        config: dict[str, Any],
        artifact_manager: ArtifactManager,
        workflow_id: str,
    ):
        """
        Initialize auto-fix resolver.

        Args:
            config: Configuration dictionary
            artifact_manager: Artifact manager for backups
            workflow_id: Current workflow execution ID
        """
        self.config = config
        self.artifact_manager = artifact_manager
        self.workflow_id = workflow_id
        self.validator = FixValidator(config)
        self.resolution_config = config.get("resolution", {})

    async def apply_auto_fix(self, issue: Issue) -> ResolutionAction:
        """
        Apply automated fix to an issue.

        Args:
            issue: Issue to fix

        Returns:
            ResolutionAction with fix results
        """
        # Check if auto-fix is enabled
        if not self.resolution_config.get("enable_auto_fix", True):
            logger.info("Auto-fix disabled in config")
            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.MANUAL_GUIDANCE,
                outcome=ResolutionOutcome.MANUAL_REQUIRED,
            )

        # Check if issue is safe to auto-fix
        if not self._is_safe_to_fix(issue):
            logger.info(f"Issue {issue.id} is not safe to auto-fix")
            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.MANUAL_GUIDANCE,
                outcome=ResolutionOutcome.MANUAL_REQUIRED,
            )

        # Identify fix pattern
        fix_pattern = self._identify_fix_pattern(issue)

        if not fix_pattern:
            logger.info(f"No auto-fix pattern found for issue {issue.id}")
            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.MANUAL_GUIDANCE,
                outcome=ResolutionOutcome.MANUAL_REQUIRED,
            )

        # Extract file path from issue
        file_path = self._extract_file_path(issue)

        if not file_path or not file_path.exists():
            logger.warning(f"Cannot determine file path for issue {issue.id}")
            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.AUTO_FIX_FAILED,
                outcome=ResolutionOutcome.MANUAL_REQUIRED,
                guidance="Cannot determine file path from issue location",
            )

        # Apply fix with safety guards
        try:
            # Create backup
            backup_path = self.artifact_manager.create_backup(
                self.workflow_id, file_path
            )

            # Apply fix
            if fix_pattern == "missing_alt_text":
                changes = self._fix_missing_alt_text(file_path)
            elif fix_pattern == "eslint_formatting":
                changes = self._fix_eslint_formatting(file_path)
            elif fix_pattern == "unused_imports":
                changes = self._fix_unused_imports(file_path)
            else:
                raise ValueError(f"Unknown fix pattern: {fix_pattern}")

            # Validate syntax
            try:
                self.validator.validate_syntax(file_path)
            except ValidationError as e:
                logger.error(f"Syntax validation failed: {e}")
                self.artifact_manager.restore_backup(backup_path, file_path)
                return ResolutionAction(
                    issue_id=issue.id,
                    workflow_execution_id=self.workflow_id,
                    action_type=ActionType.AUTO_FIX_FAILED,
                    outcome=ResolutionOutcome.ROLLBACK,
                    guidance=f"Auto-fix failed syntax validation: {e}",
                )

            # Run tests (if enabled)
            try:
                self.validator.run_related_tests(file_path)
            except ValidationError as e:
                logger.error(f"Tests failed: {e}")
                self.artifact_manager.restore_backup(backup_path, file_path)
                return ResolutionAction(
                    issue_id=issue.id,
                    workflow_execution_id=self.workflow_id,
                    action_type=ActionType.AUTO_FIX_FAILED,
                    outcome=ResolutionOutcome.ROLLBACK,
                    guidance=f"Auto-fix failed tests: {e}",
                )

            # Success!
            issue.resolution_status = ResolutionStatus.AUTO_FIXED

            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.AUTO_FIX,
                applied_changes=changes,
                verification_status=VerificationStatus.PASSED,
                outcome=ResolutionOutcome.FIXED,
            )

        except Exception as e:
            logger.error(f"Auto-fix failed for issue {issue.id}: {e}")

            # Attempt rollback
            if file_path.exists():
                try:
                    self.artifact_manager.restore_backup(backup_path, file_path)
                except Exception as rollback_error:
                    logger.error(f"Rollback failed: {rollback_error}")

            return ResolutionAction(
                issue_id=issue.id,
                workflow_execution_id=self.workflow_id,
                action_type=ActionType.AUTO_FIX_FAILED,
                outcome=ResolutionOutcome.ROLLBACK,
                guidance=f"Auto-fix failed: {str(e)}. Manual intervention required.",
            )

    def _is_safe_to_fix(self, issue: Issue) -> bool:
        """Check if issue is safe to auto-fix."""
        description_lower = issue.description.lower()

        # Check unsafe patterns
        if any(pattern in description_lower for pattern in UNSAFE_PATTERNS):
            logger.warning(f"Issue {issue.id} matches unsafe pattern, skipping auto-fix")
            return False

        return True

    def _identify_fix_pattern(self, issue: Issue) -> str | None:
        """Identify which fix pattern to apply."""
        description_lower = issue.description.lower()

        if "missing alt text" in description_lower:
            return "missing_alt_text"
        elif "eslint" in description_lower or "formatting" in description_lower:
            return "eslint_formatting"
        elif "unused import" in description_lower:
            return "unused_imports"

        return None

    def _extract_file_path(self, issue: Issue) -> Path | None:
        """Extract file path from issue context."""
        # Try to get from captured_context
        if "file_path" in issue.captured_context:
            return Path(issue.captured_context["file_path"])

        # Try to parse from location (for frontend issues)
        # This is a simplified extraction - might need enhancement
        if issue.type.value == "frontend":
            # Frontend issues typically don't have file paths
            return None

        return None

    def _fix_missing_alt_text(self, file_path: Path) -> str:
        """Fix missing alt text on images."""
        with open(file_path, "r") as f:
            content = f.read()

        # Replace img tags without alt with img tags with empty alt
        pattern = r'<img(?![^>]*\salt=)([^>]*)>'
        replacement = r'<img alt=""\1>'

        new_content = re.sub(pattern, replacement, content)

        with open(file_path, "w") as f:
            f.write(new_content)

        return f"Added alt='' to {content.count('<img')} image tags"

    def _fix_eslint_formatting(self, file_path: Path) -> str:
        """Fix ESLint formatting issues."""
        result = subprocess.run(
            ["npx", "eslint", "--fix", str(file_path)],
            capture_output=True,
            timeout=30,
        )

        if result.returncode == 0:
            return "ESLint auto-fix applied successfully"
        else:
            raise ValueError(f"ESLint fix failed: {result.stderr.decode()}")

    def _fix_unused_imports(self, file_path: Path) -> str:
        """Remove unused imports (Python only)."""
        result = subprocess.run(
            ["autoflake", "--remove-unused-variables", "--in-place", str(file_path)],
            capture_output=True,
            timeout=30,
        )

        if result.returncode == 0:
            return "Removed unused imports with autoflake"
        else:
            raise ValueError(f"autoflake failed: {result.stderr.decode()}")
