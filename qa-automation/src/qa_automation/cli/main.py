"""CLI interface for QA Automation System."""

import asyncio
import logging
import sys
from pathlib import Path
from typing import Any

import click
import yaml
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich.table import Table

from qa_automation.services.reporter import Reporter
from qa_automation.services.workflow_engine import WorkflowEngine

# Setup console for rich output
console = Console()

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)


@click.group()
@click.version_option(version="1.0.0")
def cli():
    """
    QA Automation CLI - Automated Quality Assurance for Web Applications.

    This tool helps you automatically detect, analyze, and resolve issues
    in both frontend and backend applications.
    """
    pass


@cli.command()
@click.option(
    "--config",
    type=click.Path(exists=True),
    default="config/default.yaml",
    help="Path to configuration file",
)
@click.option("--frontend-url", help="Override frontend URL from config")
@click.option("--backend-url", help="Override backend URL from config")
@click.option("--browser", type=click.Choice(["chromium", "firefox", "webkit"]), help="Browser to use")
@click.option("--headless/--no-headless", default=True, help="Run browser in headless mode")
@click.option("--parallel/--sequential", default=True, help="Run detectors in parallel")
@click.option("--dry-run", is_flag=True, help="Dry run without making changes")
@click.option("--output-dir", type=click.Path(), help="Output directory for reports and artifacts")
def run(config, frontend_url, backend_url, browser, headless, parallel, dry_run, output_dir):
    """
    Run complete QA workflow: detect, analyze, resolve, retest, and report.

    Example:
        qa-automation run --config config/default.yaml

        qa-automation run --frontend-url http://localhost:3000 --backend-url http://localhost:8000
    """
    console.print("[bold blue]QA Automation - Full Workflow[/bold blue]\n")

    # Load configuration
    config_data = _load_config(config)

    # Override with CLI parameters
    if frontend_url:
        config_data["frontend_url"] = frontend_url
    if backend_url:
        config_data["backend_url"] = backend_url
    if browser:
        config_data["browser"] = browser
    if not headless:
        config_data["headless"] = False
    if not parallel:
        config_data["parallel"] = False
    if dry_run:
        config_data.setdefault("safety", {})["dry_run"] = True
    if output_dir:
        config_data.setdefault("output", {})["artifacts_dir"] = output_dir
        config_data.setdefault("output", {})["reports_dir"] = output_dir

    # Display configuration
    _display_config(config_data)

    # Run workflow
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        task = progress.add_task("[cyan]Running QA workflow...", total=None)

        try:
            workflow_result = asyncio.run(_run_workflow(config_data))

            progress.update(task, description="[green]Workflow completed!")

            # Display results
            _display_results(workflow_result)

            # Generate reports
            progress.update(task, description="[cyan]Generating reports...")
            reporter = Reporter(config_data)
            reports = reporter.generate_all_reports(workflow_result)

            console.print("\n[bold green]Reports generated:[/bold green]")
            for report_type, report_path in reports.items():
                console.print(f"  {report_type.upper()}: {report_path}")

            # Display deployment status
            if workflow_result.deployment_ready:
                console.print("\n[bold green]✓ DEPLOYMENT READY[/bold green]")
            else:
                console.print("\n[bold red]✗ NOT READY FOR DEPLOYMENT[/bold red]")
                if workflow_result.summary:
                    console.print(
                        f"  {workflow_result.summary.critical_issues} critical, "
                        f"{workflow_result.summary.high_issues} high severity issues"
                    )

        except Exception as e:
            progress.update(task, description="[red]Workflow failed!")
            console.print(f"[bold red]Error: {e}[/bold red]")
            logger.exception("Workflow failed")
            sys.exit(1)


@cli.command()
@click.option(
    "--config",
    type=click.Path(exists=True),
    default="config/default.yaml",
    help="Path to configuration file",
)
@click.option("--frontend-url", help="Override frontend URL")
@click.option("--backend-url", help="Override backend URL")
def detect(config, frontend_url, backend_url):
    """
    Run only the detection phase (no analysis, resolution, or reporting).

    Example:
        qa-automation detect --frontend-url http://localhost:3000
    """
    console.print("[bold blue]QA Automation - Detection Only[/bold blue]\n")

    # Load configuration
    config_data = _load_config(config)

    if frontend_url:
        config_data["frontend_url"] = frontend_url
    if backend_url:
        config_data["backend_url"] = backend_url

    # Run detection
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        task = progress.add_task("[cyan]Running detection...", total=None)

        try:
            engine = WorkflowEngine(config_data)
            asyncio.run(engine.run_detection_phase())

            progress.update(task, description="[green]Detection completed!")

            # Display issues
            console.print(f"\n[bold]Issues detected: {len(engine._issues)}[/bold]")
            for issue in engine._issues[:10]:  # Show first 10
                console.print(f"  [{issue.type.value}] {issue.description}")

            if len(engine._issues) > 10:
                console.print(f"  ... and {len(engine._issues) - 10} more")

        except Exception as e:
            progress.update(task, description="[red]Detection failed!")
            console.print(f"[bold red]Error: {e}[/bold red]")
            sys.exit(1)


@cli.command()
@click.option(
    "--config",
    type=click.Path(exists=True),
    default="config/default.yaml",
    help="Path to configuration file",
)
@click.option(
    "--type",
    "scenario_type",
    type=click.Choice(["frontend", "backend"]),
    required=True,
    help="Scenario type",
)
@click.option("--id", "scenario_id", required=True, help="Scenario ID")
def test_scenario(config, scenario_type, scenario_id):
    """
    Run a single test scenario.

    Example:
        qa-automation test-scenario --type frontend --id homepage-load

        qa-automation test-scenario --type backend --id api-users-list
    """
    console.print(
        f"[bold blue]QA Automation - Test Scenario ({scenario_type}: {scenario_id})[/bold blue]\n"
    )

    # Load configuration
    config_data = _load_config(config)

    # Load scenarios
    scenarios_file = config_data.get("scenarios", {}).get(scenario_type)
    if not scenarios_file:
        console.print(f"[red]No scenarios file configured for {scenario_type}[/red]")
        sys.exit(1)

    scenarios = _load_config(scenarios_file).get("scenarios", [])
    scenario = next((s for s in scenarios if s.get("id") == scenario_id), None)

    if not scenario:
        console.print(f"[red]Scenario '{scenario_id}' not found[/red]")
        sys.exit(1)

    console.print(f"[bold]Name:[/bold] {scenario.get('name')}")
    console.print(f"[bold]Description:[/bold] {scenario.get('description')}")
    console.print()

    # Run scenario
    console.print("[yellow]Note: Single scenario execution not fully implemented yet.[/yellow]")
    console.print("[yellow]Use 'qa-automation run' for full workflow.[/yellow]")


@cli.command()
@click.option(
    "--config",
    type=click.Path(exists=True),
    default="config/default.yaml",
    help="Path to configuration file",
)
def validate(config):
    """
    Validate configuration file.

    Example:
        qa-automation validate --config config/default.yaml
    """
    console.print("[bold blue]QA Automation - Validate Configuration[/bold blue]\n")

    try:
        config_data = _load_config(config)
        console.print("[green]✓ Configuration is valid[/green]\n")

        # Display key settings
        console.print("[bold]Configuration summary:[/bold]")
        console.print(f"  Frontend URL: {config_data.get('frontend_url', 'Not set')}")
        console.print(f"  Backend URL: {config_data.get('backend_url', 'Not set')}")
        console.print(f"  Browser: {config_data.get('browser', 'chromium')}")
        console.print(f"  Parallel: {config_data.get('parallel', True)}")
        console.print(
            f"  Headless: {config_data.get('headless', True)}"
        )

    except Exception as e:
        console.print(f"[red]✗ Configuration is invalid: {e}[/red]")
        sys.exit(1)


def _load_config(config_path: str) -> dict[str, Any]:
    """Load configuration from YAML file."""
    with open(config_path) as f:
        return yaml.safe_load(f)


def _display_config(config: dict[str, Any]) -> None:
    """Display configuration summary."""
    table = Table(title="Configuration", show_header=True)
    table.add_column("Setting", style="cyan")
    table.add_column("Value", style="green")

    table.add_row("Frontend URL", str(config.get("frontend_url", "Not set")))
    table.add_row("Backend URL", str(config.get("backend_url", "Not set")))
    table.add_row("Browser", config.get("browser", "chromium"))
    table.add_row("Headless", str(config.get("headless", True)))
    table.add_row("Parallel", str(config.get("parallel", True)))
    table.add_row(
        "Dry Run", str(config.get("safety", {}).get("dry_run", False))
    )

    console.print(table)
    console.print()


def _display_results(workflow):
    """Display workflow results."""
    console.print("\n[bold]Workflow Results:[/bold]\n")

    table = Table(show_header=True)
    table.add_column("Metric", style="cyan")
    table.add_column("Value", style="green", justify="right")

    table.add_row("Workflow ID", workflow.id)
    table.add_row("Status", workflow.status.value)
    table.add_row("Duration", f"{workflow.duration_seconds}s")

    if workflow.summary:
        table.add_row("Total Issues", str(workflow.summary.total_issues_detected))
        table.add_row("Critical", str(workflow.summary.critical_issues))
        table.add_row("High", str(workflow.summary.high_issues))
        table.add_row("Medium", str(workflow.summary.medium_issues))
        table.add_row("Low", str(workflow.summary.low_issues))
        table.add_row("Auto-Fixed", str(workflow.summary.auto_fixed))
        table.add_row("Manual Required", str(workflow.summary.manual_required))
        table.add_row("Regressions", str(workflow.summary.regressions_detected))

    console.print(table)


async def _run_workflow(config: dict[str, Any]):
    """Run complete workflow."""
    engine = WorkflowEngine(config)
    return await engine.run_full_workflow()


if __name__ == "__main__":
    cli()
