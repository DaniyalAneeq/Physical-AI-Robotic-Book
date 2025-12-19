"""Artifact manager service for handling screenshots, traces, and logs."""

import logging
import shutil
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class ArtifactManager:
    """
    Manages storage and organization of test artifacts.

    Artifacts include:
    - Screenshots (PNG images)
    - Traces (Playwright .zip files)
    - Logs (text files)
    - Backups (file backups before fixes)
    """

    def __init__(self, base_dir: str = "artifacts"):
        """
        Initialize artifact manager.

        Args:
            base_dir: Base directory for storing artifacts
        """
        self.base_dir = Path(base_dir)
        self.screenshots_dir = self.base_dir / "screenshots"
        self.traces_dir = self.base_dir / "traces"
        self.logs_dir = self.base_dir / "logs"
        self.backups_dir = self.base_dir / "backups"

        # Create directories
        for directory in [
            self.screenshots_dir,
            self.traces_dir,
            self.logs_dir,
            self.backups_dir,
        ]:
            directory.mkdir(parents=True, exist_ok=True)

        logger.info(f"Artifact manager initialized: {self.base_dir}")

    def get_workflow_screenshot_dir(self, workflow_id: str) -> Path:
        """
        Get screenshot directory for a workflow.

        Args:
            workflow_id: Workflow execution ID

        Returns:
            Path to workflow's screenshot directory
        """
        path = self.screenshots_dir / workflow_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def get_workflow_trace_dir(self, workflow_id: str) -> Path:
        """
        Get trace directory for a workflow.

        Args:
            workflow_id: Workflow execution ID

        Returns:
            Path to workflow's trace directory
        """
        path = self.traces_dir / workflow_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def get_workflow_backup_dir(self, workflow_id: str) -> Path:
        """
        Get backup directory for a workflow.

        Args:
            workflow_id: Workflow execution ID

        Returns:
            Path to workflow's backup directory
        """
        path = self.backups_dir / workflow_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def save_screenshot(
        self, workflow_id: str, screenshot_name: str, screenshot_path: Path
    ) -> str:
        """
        Save a screenshot for a workflow.

        Args:
            workflow_id: Workflow execution ID
            screenshot_name: Name for the screenshot
            screenshot_path: Path to source screenshot file

        Returns:
            Relative path to saved screenshot
        """
        dest_dir = self.get_workflow_screenshot_dir(workflow_id)
        dest_path = dest_dir / f"{screenshot_name}.png"

        # Copy screenshot
        shutil.copy2(screenshot_path, dest_path)

        # Return relative path
        relative_path = str(dest_path.relative_to(self.base_dir.parent))
        logger.debug(f"Screenshot saved: {relative_path}")
        return relative_path

    def save_trace(
        self, workflow_id: str, trace_name: str, trace_path: Path
    ) -> str:
        """
        Save a trace file for a workflow.

        Args:
            workflow_id: Workflow execution ID
            trace_name: Name for the trace
            trace_path: Path to source trace file

        Returns:
            Relative path to saved trace
        """
        dest_dir = self.get_workflow_trace_dir(workflow_id)
        dest_path = dest_dir / f"{trace_name}.zip"

        # Copy trace
        shutil.copy2(trace_path, dest_path)

        # Return relative path
        relative_path = str(dest_path.relative_to(self.base_dir.parent))
        logger.debug(f"Trace saved: {relative_path}")
        return relative_path

    def get_log_file(self, workflow_id: str) -> Path:
        """
        Get log file path for a workflow.

        Args:
            workflow_id: Workflow execution ID

        Returns:
            Path to workflow's log file
        """
        return self.logs_dir / f"workflow-{workflow_id}.log"

    def create_backup(self, workflow_id: str, file_path: Path) -> Path:
        """
        Create a backup of a file before modification.

        Args:
            workflow_id: Workflow execution ID
            file_path: Path to file to backup

        Returns:
            Path to backup file
        """
        backup_dir = self.get_workflow_backup_dir(workflow_id)

        # Create backup filename (replace / with -)
        backup_name = str(file_path).replace("/", "-").replace("\\", "-")
        backup_path = backup_dir / f"{backup_name}.bak"

        # Copy file
        shutil.copy2(file_path, backup_path)

        logger.info(f"Backup created: {backup_path}")
        return backup_path

    def restore_backup(self, backup_path: Path, original_path: Path) -> None:
        """
        Restore a file from backup.

        Args:
            backup_path: Path to backup file
            original_path: Path to restore to
        """
        shutil.copy2(backup_path, original_path)
        logger.info(f"File restored from backup: {original_path}")

    def cleanup_workflow_artifacts(self, workflow_id: str) -> None:
        """
        Delete all artifacts for a workflow.

        Args:
            workflow_id: Workflow execution ID
        """
        dirs_to_remove = [
            self.screenshots_dir / workflow_id,
            self.traces_dir / workflow_id,
            self.backups_dir / workflow_id,
        ]

        for directory in dirs_to_remove:
            if directory.exists():
                shutil.rmtree(directory)
                logger.info(f"Removed artifacts: {directory}")

    def get_artifact_summary(self, workflow_id: str) -> dict[str, Any]:
        """
        Get summary of artifacts for a workflow.

        Args:
            workflow_id: Workflow execution ID

        Returns:
            Dictionary with artifact counts and paths
        """
        screenshot_dir = self.screenshots_dir / workflow_id
        trace_dir = self.traces_dir / workflow_id
        backup_dir = self.backups_dir / workflow_id

        return {
            "screenshots": {
                "count": len(list(screenshot_dir.glob("*.png")))
                if screenshot_dir.exists()
                else 0,
                "path": str(screenshot_dir.relative_to(self.base_dir.parent)),
            },
            "traces": {
                "count": len(list(trace_dir.glob("*.zip")))
                if trace_dir.exists()
                else 0,
                "path": str(trace_dir.relative_to(self.base_dir.parent)),
            },
            "backups": {
                "count": len(list(backup_dir.glob("*.bak")))
                if backup_dir.exists()
                else 0,
                "path": str(backup_dir.relative_to(self.base_dir.parent)),
            },
            "log": str(self.get_log_file(workflow_id)),
        }
