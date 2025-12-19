"""Workflow engine for orchestrating QA workflow phases."""

import asyncio
import logging
from datetime import datetime
from typing import Any

from qa_automation.analyzers.grouping import GroupingAnalyzer
from qa_automation.analyzers.root_cause import RootCauseAnalyzer
from qa_automation.analyzers.severity import SeverityAnalyzer
from qa_automation.detectors.backend import BackendDetector
from qa_automation.detectors.frontend import FrontendDetector
from qa_automation.detectors.performance import PerformanceDetector
from qa_automation.models.issue import Issue, SeverityLevel
from qa_automation.models.resolution import ResolutionAction, ActionType, ResolutionOutcome
from qa_automation.models.test_result import TestResult
from qa_automation.models.workflow import (
    WorkflowExecution,
    WorkflowStatus,
    PhaseResult,
    WorkflowSummary,
)
from qa_automation.resolvers.auto_fix import AutoFixResolver
from qa_automation.resolvers.guidance import GuidanceGenerator
from qa_automation.services.artifact_manager import ArtifactManager

logger = logging.getLogger(__name__)


class WorkflowEngine:
    """
    Main workflow orchestration engine.

    Orchestrates all phases:
    1. Detection (frontend + backend + performance)
    2. Analysis (severity, root cause, grouping)
    3. Resolution (auto-fix, guidance)
    4. Re-testing (regression detection)
    5. Reporting (JSON, HTML, deployment readiness)
    """

    def __init__(self, config: dict[str, Any]):
        """
        Initialize workflow engine.

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.artifact_manager = ArtifactManager(
            config.get("output", {}).get("artifacts_dir", "artifacts")
        )

        # Create workflow execution
        self.workflow = WorkflowExecution(config=config)
        self.workflow.status = WorkflowStatus.PENDING

        # Store actual objects (not just IDs) for processing
        self._test_results: list[TestResult] = []
        self._issues: list[Issue] = []
        self._resolution_actions: list[ResolutionAction] = []

        logger.info(f"Workflow engine initialized: {self.workflow.id}")

    async def run_detection_phase(self) -> PhaseResult:
        """
        Execute detection phase: run frontend, backend, and performance detectors.

        Returns:
            PhaseResult for detection phase
        """
        logger.info("Starting detection phase")
        phase_start = datetime.utcnow()

        all_test_results: list[TestResult] = []
        all_issues: list[Issue] = []

        try:
            # Check if parallel execution is enabled
            parallel = self.config.get("parallel", True)
            skip_frontend = self.config.get("skip_frontend", False)

            if parallel and not skip_frontend:
                # Run frontend and backend detectors in parallel
                logger.info("Running detectors in parallel")

                frontend_task = self._run_frontend_detector()
                backend_task = self._run_backend_detector()

                results = await asyncio.gather(
                    frontend_task, backend_task, return_exceptions=True
                )

                # Process frontend results
                if isinstance(results[0], Exception):
                    logger.error(f"Frontend detector failed: {results[0]}")
                else:
                    frontend_results, frontend_issues = results[0]
                    all_test_results.extend(frontend_results)
                    all_issues.extend(frontend_issues)

                # Process backend results
                if isinstance(results[1], Exception):
                    logger.error(f"Backend detector failed: {results[1]}")
                else:
                    backend_results, backend_issues = results[1]
                    all_test_results.extend(backend_results)
                    all_issues.extend(backend_issues)

            else:
                # Run detectors sequentially
                logger.info("Running detectors sequentially")

                # Frontend detection (skip if configured)
                if not skip_frontend:
                    try:
                        frontend_results, frontend_issues = await self._run_frontend_detector()
                        all_test_results.extend(frontend_results)
                        all_issues.extend(frontend_issues)
                    except Exception as e:
                        logger.error(f"Frontend detector failed: {e}")
                else:
                    logger.info("Skipping frontend tests (skip_frontend=true)")

                # Backend detection
                try:
                    backend_results, backend_issues = await self._run_backend_detector()
                    all_test_results.extend(backend_results)
                    all_issues.extend(backend_issues)
                except Exception as e:
                    logger.error(f"Backend detector failed: {e}")

            # Run performance analysis on collected results
            performance_detector = PerformanceDetector(self.config)
            performance_issues = performance_detector.analyze_test_results(
                all_test_results
            )
            all_issues.extend(performance_issues)

            # Store results in workflow (both IDs and actual objects)
            self._test_results = all_test_results
            self._issues = all_issues
            self.workflow.test_results = [r.id for r in all_test_results]
            self.workflow.detected_issues = [i.id for i in all_issues]

            # Calculate phase duration
            phase_duration = int(
                (datetime.utcnow() - phase_start).total_seconds()
            )

            # Count successful/failed tests
            successful = sum(
                1 for r in all_test_results if r.status.value == "passed"
            )
            failed = len(all_test_results) - successful

            # Create phase result
            phase_result = PhaseResult(
                phase_name="detection",
                duration_seconds=phase_duration,
                status="completed",
                items_processed=len(all_test_results),
                items_successful=successful,
                items_failed=failed,
            )

            self.workflow.phase_results.append(phase_result)

            logger.info(
                f"Detection phase completed: {len(all_issues)} issues detected, "
                f"{successful}/{len(all_test_results)} tests passed"
            )

            return phase_result

        except Exception as e:
            logger.error(f"Detection phase failed: {e}")
            phase_duration = int(
                (datetime.utcnow() - phase_start).total_seconds()
            )

            phase_result = PhaseResult(
                phase_name="detection",
                duration_seconds=phase_duration,
                status="failed",
                items_processed=0,
                items_successful=0,
                items_failed=0,
            )

            self.workflow.phase_results.append(phase_result)
            raise

    async def _run_frontend_detector(
        self,
    ) -> tuple[list[TestResult], list[Issue]]:
        """
        Run frontend detector.

        Returns:
            Tuple of (test_results, detected_issues)
        """
        logger.info("Running frontend detector")

        detector = FrontendDetector(
            self.config, self.artifact_manager, self.workflow.id
        )

        return await detector.detect()

    async def _run_backend_detector(
        self,
    ) -> tuple[list[TestResult], list[Issue]]:
        """
        Run backend detector.

        Returns:
            Tuple of (test_results, detected_issues)
        """
        logger.info("Running backend detector")

        detector = BackendDetector(self.config, self.workflow.id)

        return await detector.detect()

    async def run_analysis_phase(self, issues: list[Issue]) -> PhaseResult:
        """
        Execute analysis phase: assign severity, identify root causes, group issues.

        Args:
            issues: List of detected issues

        Returns:
            PhaseResult for analysis phase
        """
        logger.info("Starting analysis phase")
        phase_start = datetime.utcnow()

        try:
            # Initialize analyzers
            severity_analyzer = SeverityAnalyzer(self.config)
            root_cause_analyzer = RootCauseAnalyzer(self.config)
            grouping_analyzer = GroupingAnalyzer(self.config)

            # Step 1: Assign severity
            issues = severity_analyzer.analyze_issues(issues)

            # Step 2: Identify root causes
            issues = root_cause_analyzer.analyze_issues(issues)

            # Step 3: Group related issues
            issues = grouping_analyzer.analyze_issues(issues)

            # Calculate phase duration
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            # Create phase result
            phase_result = PhaseResult(
                phase_name="analysis",
                duration_seconds=phase_duration,
                status="completed",
                items_processed=len(issues),
                items_successful=len(issues),
                items_failed=0,
            )

            self.workflow.phase_results.append(phase_result)

            logger.info(
                f"Analysis phase completed: {len(issues)} issues analyzed "
                f"in {phase_duration}s"
            )

            return phase_result

        except Exception as e:
            logger.error(f"Analysis phase failed: {e}")
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            phase_result = PhaseResult(
                phase_name="analysis",
                duration_seconds=phase_duration,
                status="failed",
                items_processed=0,
                items_successful=0,
                items_failed=0,
            )

            self.workflow.phase_results.append(phase_result)
            raise

    async def run_resolution_phase(self, issues: list[Issue]) -> PhaseResult:
        """
        Execute resolution phase: apply auto-fixes and generate guidance.

        Args:
            issues: List of analyzed issues

        Returns:
            PhaseResult for resolution phase
        """
        logger.info("Starting resolution phase")
        phase_start = datetime.utcnow()

        try:
            # Initialize resolvers
            auto_fix_resolver = AutoFixResolver(
                self.config, self.artifact_manager, self.workflow.id
            )
            guidance_generator = GuidanceGenerator(self.config)

            resolution_actions: list[ResolutionAction] = []

            # Process each issue
            for issue in issues:
                # Attempt auto-fix
                action = await auto_fix_resolver.apply_auto_fix(issue)

                # If auto-fix not applicable, generate guidance
                if action.action_type == ActionType.MANUAL_GUIDANCE:
                    action.guidance = guidance_generator.generate_resolution_guidance(issue)

                resolution_actions.append(action)
                issue.resolution_action_id = action.id

            # Store resolution actions
            self._resolution_actions = resolution_actions
            self.workflow.resolution_actions = [a.id for a in resolution_actions]

            # Calculate summary statistics
            auto_fixed = sum(
                1 for a in resolution_actions if a.outcome == ResolutionOutcome.FIXED
            )
            manual_required = sum(
                1 for a in resolution_actions
                if a.outcome == ResolutionOutcome.MANUAL_REQUIRED
            )
            rollback = sum(
                1 for a in resolution_actions if a.outcome == ResolutionOutcome.ROLLBACK
            )

            # Update workflow summary
            if self.workflow.summary:
                self.workflow.summary.auto_fixed = auto_fixed
                self.workflow.summary.manual_required = manual_required

            # Calculate phase duration
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            # Create phase result
            phase_result = PhaseResult(
                phase_name="resolution",
                duration_seconds=phase_duration,
                status="completed",
                items_processed=len(issues),
                items_successful=auto_fixed,
                items_failed=rollback,
            )

            self.workflow.phase_results.append(phase_result)

            logger.info(
                f"Resolution phase completed: {auto_fixed} auto-fixed, "
                f"{manual_required} require manual intervention, "
                f"{rollback} failed/rolled back"
            )

            return phase_result

        except Exception as e:
            logger.error(f"Resolution phase failed: {e}")
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            phase_result = PhaseResult(
                phase_name="resolution",
                duration_seconds=phase_duration,
                status="failed",
                items_processed=0,
                items_successful=0,
                items_failed=0,
            )

            self.workflow.phase_results.append(phase_result)
            raise

    async def run_retest_phase(
        self, original_issues: list[Issue], iteration: int = 1
    ) -> PhaseResult:
        """
        Execute re-testing phase: re-run detection and check for regressions.

        Args:
            original_issues: Issues detected before fixes
            iteration: Current iteration number

        Returns:
            PhaseResult for re-testing phase
        """
        logger.info(f"Starting re-testing phase (iteration {iteration})")
        phase_start = datetime.utcnow()

        try:
            # Re-run detection phase
            await self.run_detection_phase()

            # Get newly detected issues
            retest_issues = self._issues

            # Compare with original issues
            fixed_issues = self._identify_fixed_issues(original_issues, retest_issues)
            still_present = self._identify_still_present(original_issues, retest_issues)
            regressions = self._identify_regressions(original_issues, retest_issues)

            # Calculate regression score
            regression_score = (
                len(regressions) / len(original_issues) if original_issues else 0.0
            )

            # Update workflow summary
            if self.workflow.summary:
                self.workflow.summary.regressions_detected = len(regressions)

            # Calculate phase duration
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            # Create phase result
            phase_result = PhaseResult(
                phase_name=f"retest_iteration_{iteration}",
                duration_seconds=phase_duration,
                status="completed",
                items_processed=len(original_issues),
                items_successful=len(fixed_issues),
                items_failed=len(still_present) + len(regressions),
            )

            self.workflow.phase_results.append(phase_result)

            logger.info(
                f"Re-testing phase completed: {len(fixed_issues)} fixed, "
                f"{len(still_present)} still present, "
                f"{len(regressions)} regressions (score: {regression_score:.2%})"
            )

            return phase_result

        except Exception as e:
            logger.error(f"Re-testing phase failed: {e}")
            phase_duration = int((datetime.utcnow() - phase_start).total_seconds())

            phase_result = PhaseResult(
                phase_name=f"retest_iteration_{iteration}",
                duration_seconds=phase_duration,
                status="failed",
                items_processed=0,
                items_successful=0,
                items_failed=0,
            )

            self.workflow.phase_results.append(phase_result)
            raise

    def _issue_signature(self, issue: Issue) -> str:
        """
        Generate unique signature for issue comparison.

        Args:
            issue: Issue to generate signature for

        Returns:
            Unique signature string
        """
        return f"{issue.type.value}:{issue.location}:{issue.description[:100]}"

    def _identify_fixed_issues(
        self, original: list[Issue], retest: list[Issue]
    ) -> list[Issue]:
        """Identify issues that were fixed (present in original, not in retest)."""
        retest_signatures = {self._issue_signature(i) for i in retest}
        return [i for i in original if self._issue_signature(i) not in retest_signatures]

    def _identify_still_present(
        self, original: list[Issue], retest: list[Issue]
    ) -> list[Issue]:
        """Identify issues that are still present."""
        original_signatures = {self._issue_signature(i) for i in original}
        return [i for i in retest if self._issue_signature(i) in original_signatures]

    def _identify_regressions(
        self, original: list[Issue], retest: list[Issue]
    ) -> list[Issue]:
        """Identify new issues introduced (regressions)."""
        original_signatures = {self._issue_signature(i) for i in original}
        return [i for i in retest if self._issue_signature(i) not in original_signatures]

    def _should_continue_retest_loop(
        self, iteration: int, regression_score: float
    ) -> bool:
        """
        Determine if re-testing loop should continue.

        Args:
            iteration: Current iteration number
            regression_score: Ratio of new issues to original issues

        Returns:
            True if loop should continue
        """
        max_iterations = self.config.get("safety", {}).get("max_retest_iterations", 3)
        regression_threshold = 0.05  # 5%

        return iteration < max_iterations and regression_score > regression_threshold

    async def run_full_workflow(self) -> WorkflowExecution:
        """
        Run complete workflow (all phases).

        Returns:
            Completed WorkflowExecution
        """
        logger.info(f"Starting full workflow: {self.workflow.id}")
        self.workflow.status = WorkflowStatus.RUNNING
        self.workflow.start_time = datetime.utcnow()

        try:
            # Phase 1: Detection
            logger.info("=== Phase 1: Detection ===")
            await self.run_detection_phase()

            # Phase 2: Analysis
            if self._issues:
                logger.info("=== Phase 2: Analysis ===")
                await self.run_analysis_phase(self._issues)

                # Calculate initial summary
                self.workflow.summary = self._calculate_summary(self._issues)

                # Determine initial deployment readiness
                self.workflow.deployment_ready = self._is_deployment_ready(self._issues)

                # Phase 3: Resolution
                logger.info("=== Phase 3: Resolution ===")
                original_issues = self._issues.copy()
                await self.run_resolution_phase(self._issues)

                # Phase 4: Re-testing (optional, based on config)
                enable_retest = self.config.get("safety", {}).get(
                    "enable_retest", True
                )
                if enable_retest and self._resolution_actions:
                    logger.info("=== Phase 4: Re-testing ===")

                    iteration = 1
                    current_issues = original_issues

                    while True:
                        # Run re-test
                        await self.run_retest_phase(current_issues, iteration)

                        # Check if we should continue
                        retest_issues = self._issues
                        regressions = self._identify_regressions(
                            current_issues, retest_issues
                        )
                        regression_score = (
                            len(regressions) / len(current_issues)
                            if current_issues
                            else 0.0
                        )

                        if not self._should_continue_retest_loop(
                            iteration, regression_score
                        ):
                            logger.info(
                                f"Re-testing loop terminated after {iteration} iteration(s)"
                            )
                            break

                        # Update for next iteration
                        current_issues = retest_issues
                        iteration += 1

                    # Re-calculate final deployment readiness
                    self.workflow.deployment_ready = self._is_deployment_ready(
                        self._issues
                    )
            else:
                logger.info("No issues detected, skipping analysis and resolution phases")

            # Mark workflow as completed
            self.workflow.status = WorkflowStatus.COMPLETED
            self.workflow.end_time = datetime.utcnow()
            self.workflow.duration_seconds = int(
                (self.workflow.end_time - self.workflow.start_time).total_seconds()
            )

            logger.info(
                f"Workflow completed: {self.workflow.id} "
                f"({self.workflow.duration_seconds}s, "
                f"{len(self.workflow.detected_issues)} issues detected)"
            )

            return self.workflow

        except Exception as e:
            logger.error(f"Workflow failed: {e}")
            self.workflow.status = WorkflowStatus.FAILED
            self.workflow.end_time = datetime.utcnow()
            self.workflow.duration_seconds = int(
                (self.workflow.end_time - self.workflow.start_time).total_seconds()
            )
            raise

    def _calculate_summary(self, issues: list[Issue]) -> WorkflowSummary:
        """
        Calculate workflow summary from issues.

        Args:
            issues: List of analyzed issues

        Returns:
            WorkflowSummary
        """
        return WorkflowSummary(
            total_issues_detected=len(issues),
            critical_issues=sum(
                1 for i in issues if i.severity == SeverityLevel.CRITICAL
            ),
            high_issues=sum(1 for i in issues if i.severity == SeverityLevel.HIGH),
            medium_issues=sum(1 for i in issues if i.severity == SeverityLevel.MEDIUM),
            low_issues=sum(1 for i in issues if i.severity == SeverityLevel.LOW),
            auto_fixed=0,  # Will be updated in resolution phase
            manual_required=0,  # Will be updated in resolution phase
            regressions_detected=0,  # Will be updated in retest phase
        )

    def _is_deployment_ready(self, issues: list[Issue]) -> bool:
        """
        Determine if application is ready for deployment.

        Args:
            issues: List of analyzed issues

        Returns:
            True if deployment ready (no critical or high issues)
        """
        critical_count = sum(
            1 for i in issues if i.severity == SeverityLevel.CRITICAL
        )
        high_count = sum(1 for i in issues if i.severity == SeverityLevel.HIGH)

        return critical_count == 0 and high_count == 0
