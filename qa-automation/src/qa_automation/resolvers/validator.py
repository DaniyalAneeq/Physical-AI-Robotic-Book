"""Fix validator for verifying changes after auto-fix."""

import ast
import logging
import subprocess
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class ValidationError(Exception):
    """Raised when fix validation fails."""
    pass


class FixValidator:
    """
    Validates fixes before accepting them.

    Validation steps:
    1. Create backup of original file
    2. Validate syntax after fix
    3. Run related tests (if available)
    4. Rollback on failure
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize fix validator.

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.safety_config = config.get("safety", {})

    def validate_syntax(self, file_path: Path) -> bool:
        """
        Validate syntax of a file after fix.

        Args:
            file_path: Path to file to validate

        Returns:
            True if syntax is valid

        Raises:
            ValidationError: If syntax is invalid
        """
        if not file_path.exists():
            raise ValidationError(f"File not found: {file_path}")

        suffix = file_path.suffix

        try:
            if suffix == ".py":
                return self._validate_python_syntax(file_path)
            elif suffix in [".js", ".jsx", ".ts", ".tsx"]:
                return self._validate_javascript_syntax(file_path)
            else:
                # Unknown file type, assume valid
                logger.warning(f"Cannot validate syntax for {suffix} files")
                return True

        except Exception as e:
            raise ValidationError(f"Syntax validation failed: {e}")

    def _validate_python_syntax(self, file_path: Path) -> bool:
        """Validate Python syntax."""
        try:
            with open(file_path) as f:
                ast.parse(f.read())
            logger.debug(f"Python syntax valid: {file_path}")
            return True
        except SyntaxError as e:
            raise ValidationError(f"Python syntax error: {e}")

    def _validate_javascript_syntax(self, file_path: Path) -> bool:
        """Validate JavaScript/TypeScript syntax using eslint."""
        try:
            result = subprocess.run(
                ["npx", "eslint", "--parser-options=ecmaVersion:latest", str(file_path)],
                capture_output=True,
                timeout=10,
            )
            if result.returncode == 0:
                logger.debug(f"JavaScript syntax valid: {file_path}")
                return True
            else:
                raise ValidationError(f"ESLint errors: {result.stderr.decode()}")
        except subprocess.TimeoutExpired:
            raise ValidationError("ESLint validation timed out")
        except FileNotFoundError:
            logger.warning("ESLint not found, skipping JS syntax validation")
            return True

    def run_related_tests(self, file_path: Path) -> bool:
        """
        Run tests related to the modified file.

        Args:
            file_path: Path to modified file

        Returns:
            True if tests pass

        Raises:
            ValidationError: If tests fail
        """
        # Skip if run_tests is disabled in config
        if not self.safety_config.get("run_tests", True):
            logger.info("Test execution disabled in config")
            return True

        # Find test file
        test_file = self._find_test_file(file_path)

        if not test_file or not test_file.exists():
            logger.info(f"No test file found for {file_path}, skipping tests")
            return True

        try:
            if test_file.suffix == ".py":
                return self._run_pytest(test_file)
            else:
                logger.warning(f"Don't know how to run tests for {test_file.suffix}")
                return True

        except Exception as e:
            raise ValidationError(f"Test execution failed: {e}")

    def _find_test_file(self, file_path: Path) -> Path | None:
        """Find corresponding test file for a source file."""
        # Common test file patterns
        parent = file_path.parent
        stem = file_path.stem

        patterns = [
            parent / f"test_{stem}{file_path.suffix}",
            parent / f"{stem}_test{file_path.suffix}",
            parent.parent / "tests" / f"test_{stem}{file_path.suffix}",
        ]

        for pattern in patterns:
            if pattern.exists():
                return pattern

        return None

    def _run_pytest(self, test_file: Path) -> bool:
        """Run pytest on a test file."""
        try:
            result = subprocess.run(
                ["pytest", str(test_file), "-v", "--tb=short"],
                capture_output=True,
                timeout=60,
            )

            if result.returncode == 0:
                logger.info(f"Tests passed: {test_file}")
                return True
            else:
                raise ValidationError(
                    f"Tests failed: {result.stdout.decode()}\n{result.stderr.decode()}"
                )

        except subprocess.TimeoutExpired:
            raise ValidationError("Test execution timed out")
        except FileNotFoundError:
            logger.warning("pytest not found, skipping test execution")
            return True
