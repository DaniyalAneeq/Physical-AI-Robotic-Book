# QA Automation - Automated Quality Assurance Workflow

An end-to-end automated quality assurance system that detects, analyzes, resolves, and re-tests issues in both frontend (browser) and backend (API) applications before production deployment.

## Features

- **Automated Issue Detection**: Browser automation (Playwright) for frontend testing, HTTP client testing for backend API validation
- **Intelligent Analysis**: Severity assignment, root cause identification, and issue grouping
- **Automated Resolution**: Safe auto-fixes with validation and rollback capabilities
- **Continuous Validation**: Re-testing with regression detection
- **Comprehensive Reporting**: JSON, HTML dashboards, and deployment readiness assessments

## Quick Start

### Installation

```bash
# Install the package
pip install -e ./qa-automation

# Install Playwright browsers
playwright install chromium
```

### Basic Usage

```bash
# Run complete workflow
qa-automation run

# Dry run (simulate without applying fixes)
qa-automation run --dry-run

# Custom configuration
qa-automation run --config path/to/config.yaml
```

### Configuration

Create a `.env` file with test credentials:

```bash
TEST_AUTH_EMAIL=test.user@example.com
TEST_AUTH_PASSWORD=SecureTestPassword123!
```

Configure workflow in `config/default.yaml`:

```yaml
frontend_url: http://localhost:3000
backend_url: https://e-book-physical-ai-humanoid-robotics.onrender.com
browser: chromium
headless: true
parallel: true
max_concurrent: 10
```

## Project Structure

```
qa-automation/
├── src/
│   ├── detectors/      # Issue detection modules
│   ├── analyzers/      # Issue analysis modules
│   ├── resolvers/      # Issue resolution modules
│   ├── models/         # Data models (Pydantic)
│   ├── services/       # Core workflow services
│   ├── cli/            # Command-line interface
│   └── lib/            # Public API
├── tests/              # Test suite
├── config/             # Configuration files
└── pyproject.toml      # Project dependencies
```

## Documentation

- [Quickstart Guide](../specs/001-automated-qa-workflow/quickstart.md)
- [Data Model Reference](../specs/001-automated-qa-workflow/data-model.md)
- [Technical Plan](../specs/001-automated-qa-workflow/plan.md)
- [Implementation Tasks](../specs/001-automated-qa-workflow/tasks.md)

## Requirements

- Python 3.11+
- Linux or macOS (Windows WSL2 supported)
- 4GB+ RAM (8GB recommended for parallel testing)

## CI/CD Integration

See [GitHub Actions example](../.github/workflows/qa-workflow.yml) for CI/CD integration.

## License

MIT License
