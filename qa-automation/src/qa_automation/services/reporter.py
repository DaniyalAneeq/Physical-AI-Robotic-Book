"""Reporter service for generating QA workflow reports."""

import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any

from qa_automation.models.workflow import WorkflowExecution

logger = logging.getLogger(__name__)


class Reporter:
    """
    Generates various report formats for workflow executions.

    Supported formats:
    - JSON: Complete workflow data
    - HTML: Interactive dashboard
    - Text: Deployment readiness summary
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize reporter.

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.output_config = config.get("output", {})
        self.reports_dir = Path(self.output_config.get("reports_dir", "reports"))
        self.reports_dir.mkdir(parents=True, exist_ok=True)

    def generate_json_report(self, workflow: WorkflowExecution) -> str:
        """
        Generate JSON report with complete workflow data.

        Args:
            workflow: Completed workflow execution

        Returns:
            Path to generated JSON file
        """
        report_path = self.reports_dir / f"workflow-{workflow.id}.json"

        logger.info(f"Generating JSON report: {report_path}")

        # Serialize workflow to JSON
        with open(report_path, "w") as f:
            f.write(workflow.model_dump_json(indent=2))

        logger.info(f"JSON report generated: {report_path}")

        return str(report_path)

    def generate_html_report(self, workflow: WorkflowExecution) -> str:
        """
        Generate HTML dashboard report.

        Args:
            workflow: Completed workflow execution

        Returns:
            Path to generated HTML file
        """
        report_path = self.reports_dir / f"dashboard-{workflow.id}.html"

        logger.info(f"Generating HTML report: {report_path}")

        # Generate HTML content
        html_content = self._generate_html_dashboard(workflow)

        with open(report_path, "w") as f:
            f.write(html_content)

        logger.info(f"HTML report generated: {report_path}")

        return str(report_path)

    def generate_text_report(self, workflow: WorkflowExecution) -> str:
        """
        Generate deployment readiness text report.

        Args:
            workflow: Completed workflow execution

        Returns:
            Path to generated text file
        """
        report_path = self.reports_dir / f"deployment-{workflow.id}.txt"

        logger.info(f"Generating text report: {report_path}")

        # Generate text content
        text_content = self._generate_text_summary(workflow)

        with open(report_path, "w") as f:
            f.write(text_content)

        logger.info(f"Text report generated: {report_path}")

        return str(report_path)

    def _generate_html_dashboard(self, workflow: WorkflowExecution) -> str:
        """
        Generate HTML dashboard content.

        Args:
            workflow: Workflow execution

        Returns:
            HTML content as string
        """
        summary = workflow.summary
        status_color = "green" if workflow.deployment_ready else "red"
        status_text = "READY" if workflow.deployment_ready else "NOT READY"

        # Calculate phase statistics
        phase_stats = []
        for phase in workflow.phase_results:
            phase_stats.append(
                {
                    "name": phase.phase_name,
                    "duration": phase.duration_seconds,
                    "status": phase.status,
                    "processed": phase.items_processed,
                    "successful": phase.items_successful,
                    "failed": phase.items_failed,
                }
            )

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>QA Workflow Report - {workflow.id}</title>
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}

        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
            background: #f5f7fa;
            color: #2c3e50;
            padding: 20px;
        }}

        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            padding: 30px;
        }}

        .header {{
            border-bottom: 2px solid #e1e8ed;
            padding-bottom: 20px;
            margin-bottom: 30px;
        }}

        .header h1 {{
            font-size: 32px;
            margin-bottom: 10px;
        }}

        .header .meta {{
            color: #657786;
            font-size: 14px;
        }}

        .deployment-status {{
            background: {status_color};
            color: white;
            padding: 20px;
            border-radius: 8px;
            text-align: center;
            margin-bottom: 30px;
        }}

        .deployment-status h2 {{
            font-size: 24px;
            margin-bottom: 10px;
        }}

        .deployment-status p {{
            font-size: 16px;
            opacity: 0.9;
        }}

        .summary {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }}

        .summary-card {{
            background: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            border-left: 4px solid #3498db;
        }}

        .summary-card.critical {{
            border-left-color: #e74c3c;
        }}

        .summary-card.high {{
            border-left-color: #e67e22;
        }}

        .summary-card.medium {{
            border-left-color: #f39c12;
        }}

        .summary-card.low {{
            border-left-color: #95a5a6;
        }}

        .summary-card h3 {{
            font-size: 14px;
            color: #657786;
            margin-bottom: 10px;
            text-transform: uppercase;
        }}

        .summary-card .number {{
            font-size: 36px;
            font-weight: bold;
        }}

        .phases {{
            margin-bottom: 30px;
        }}

        .phases h2 {{
            font-size: 20px;
            margin-bottom: 20px;
        }}

        .phase {{
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 10px;
        }}

        .phase-header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }}

        .phase-name {{
            font-weight: bold;
            font-size: 16px;
        }}

        .phase-status {{
            padding: 4px 12px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
            text-transform: uppercase;
        }}

        .phase-status.completed {{
            background: #d4edda;
            color: #155724;
        }}

        .phase-status.failed {{
            background: #f8d7da;
            color: #721c24;
        }}

        .phase-stats {{
            display: flex;
            gap: 20px;
            font-size: 14px;
            color: #657786;
        }}

        .phase-stats span {{
            display: inline-block;
        }}

        .footer {{
            text-align: center;
            padding-top: 20px;
            border-top: 2px solid #e1e8ed;
            color: #657786;
            font-size: 14px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>QA Workflow Report</h1>
            <div class="meta">
                <strong>Workflow ID:</strong> {workflow.id}<br>
                <strong>Started:</strong> {workflow.start_time.strftime('%Y-%m-%d %H:%M:%S') if workflow.start_time else 'N/A'}<br>
                <strong>Completed:</strong> {workflow.end_time.strftime('%Y-%m-%d %H:%M:%S') if workflow.end_time else 'N/A'}<br>
                <strong>Duration:</strong> {workflow.duration_seconds}s
            </div>
        </div>

        <div class="deployment-status">
            <h2>Deployment Status: {status_text}</h2>
            <p>{self._get_deployment_message(workflow)}</p>
        </div>

        <div class="summary">
            <div class="summary-card">
                <h3>Total Issues</h3>
                <div class="number">{summary.total_issues_detected if summary else 0}</div>
            </div>
            <div class="summary-card critical">
                <h3>Critical</h3>
                <div class="number">{summary.critical_issues if summary else 0}</div>
            </div>
            <div class="summary-card high">
                <h3>High</h3>
                <div class="number">{summary.high_issues if summary else 0}</div>
            </div>
            <div class="summary-card medium">
                <h3>Medium</h3>
                <div class="number">{summary.medium_issues if summary else 0}</div>
            </div>
            <div class="summary-card low">
                <h3>Low</h3>
                <div class="number">{summary.low_issues if summary else 0}</div>
            </div>
            <div class="summary-card">
                <h3>Auto-Fixed</h3>
                <div class="number">{summary.auto_fixed if summary else 0}</div>
            </div>
            <div class="summary-card">
                <h3>Manual Required</h3>
                <div class="number">{summary.manual_required if summary else 0}</div>
            </div>
            <div class="summary-card">
                <h3>Regressions</h3>
                <div class="number">{summary.regressions_detected if summary else 0}</div>
            </div>
        </div>

        <div class="phases">
            <h2>Phase Results</h2>
            {self._generate_phase_html(phase_stats)}
        </div>

        <div class="footer">
            <p>Generated by QA Automation System | {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        </div>
    </div>
</body>
</html>
"""

        return html

    def _generate_phase_html(self, phase_stats: list[dict]) -> str:
        """Generate HTML for phase statistics."""
        html_parts = []

        for phase in phase_stats:
            status_class = phase["status"]
            html_parts.append(
                f"""
            <div class="phase">
                <div class="phase-header">
                    <div class="phase-name">{phase['name']}</div>
                    <div class="phase-status {status_class}">{phase['status']}</div>
                </div>
                <div class="phase-stats">
                    <span><strong>Duration:</strong> {phase['duration']}s</span>
                    <span><strong>Processed:</strong> {phase['processed']}</span>
                    <span><strong>Successful:</strong> {phase['successful']}</span>
                    <span><strong>Failed:</strong> {phase['failed']}</span>
                </div>
            </div>
            """
            )

        return "".join(html_parts)

    def _get_deployment_message(self, workflow: WorkflowExecution) -> str:
        """Get deployment readiness message."""
        if workflow.deployment_ready:
            return "No critical or high severity issues detected. Application is ready for deployment."
        else:
            summary = workflow.summary
            if summary:
                critical = summary.critical_issues
                high = summary.high_issues
                return f"Deployment blocked: {critical} critical issue(s) and {high} high severity issue(s) detected."
            return "Deployment blocked due to detected issues."

    def _generate_text_summary(self, workflow: WorkflowExecution) -> str:
        """
        Generate text summary of workflow.

        Args:
            workflow: Workflow execution

        Returns:
            Text content
        """
        summary = workflow.summary
        lines = []

        lines.append("=" * 60)
        lines.append("QA WORKFLOW DEPLOYMENT READINESS REPORT")
        lines.append("=" * 60)
        lines.append("")

        lines.append(f"Workflow ID: {workflow.id}")
        lines.append(
            f"Started: {workflow.start_time.strftime('%Y-%m-%d %H:%M:%S') if workflow.start_time else 'N/A'}"
        )
        lines.append(
            f"Completed: {workflow.end_time.strftime('%Y-%m-%d %H:%M:%S') if workflow.end_time else 'N/A'}"
        )
        lines.append(f"Duration: {workflow.duration_seconds}s")
        lines.append(f"Status: {workflow.status.value}")
        lines.append("")

        lines.append("-" * 60)
        lines.append("DEPLOYMENT STATUS")
        lines.append("-" * 60)
        lines.append("")

        if workflow.deployment_ready:
            lines.append("✓ READY FOR DEPLOYMENT")
            lines.append("")
            lines.append("No critical or high severity issues detected.")
        else:
            lines.append("✗ NOT READY FOR DEPLOYMENT")
            lines.append("")
            if summary:
                lines.append(f"Critical issues: {summary.critical_issues}")
                lines.append(f"High severity issues: {summary.high_issues}")
                lines.append("")
                lines.append("Please resolve all critical and high severity issues")
                lines.append("before deploying to production.")

        lines.append("")
        lines.append("-" * 60)
        lines.append("ISSUE SUMMARY")
        lines.append("-" * 60)
        lines.append("")

        if summary:
            lines.append(f"Total issues detected: {summary.total_issues_detected}")
            lines.append(f"  Critical: {summary.critical_issues}")
            lines.append(f"  High: {summary.high_issues}")
            lines.append(f"  Medium: {summary.medium_issues}")
            lines.append(f"  Low: {summary.low_issues}")
            lines.append("")
            lines.append(f"Auto-fixed: {summary.auto_fixed}")
            lines.append(f"Manual intervention required: {summary.manual_required}")
            lines.append(f"Regressions detected: {summary.regressions_detected}")

        lines.append("")
        lines.append("-" * 60)
        lines.append("PHASE RESULTS")
        lines.append("-" * 60)
        lines.append("")

        for phase in workflow.phase_results:
            lines.append(f"{phase.phase_name}:")
            lines.append(f"  Status: {phase.status}")
            lines.append(f"  Duration: {phase.duration_seconds}s")
            lines.append(f"  Processed: {phase.items_processed}")
            lines.append(f"  Successful: {phase.items_successful}")
            lines.append(f"  Failed: {phase.items_failed}")
            lines.append("")

        lines.append("=" * 60)
        lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("=" * 60)

        return "\n".join(lines)

    def generate_all_reports(self, workflow: WorkflowExecution) -> dict[str, str]:
        """
        Generate all report formats.

        Args:
            workflow: Completed workflow execution

        Returns:
            Dictionary mapping report type to file path
        """
        logger.info("Generating all reports")

        reports = {
            "json": self.generate_json_report(workflow),
            "html": self.generate_html_report(workflow),
            "text": self.generate_text_report(workflow),
        }

        logger.info(f"All reports generated: {reports}")

        return reports
