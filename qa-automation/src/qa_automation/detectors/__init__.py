"""
Issue detection modules.

Detectors scan applications and identify issues:
- Frontend: Browser automation with Playwright
- Backend: API testing with httpx
- Performance: Metrics collection and threshold validation
"""

from qa_automation.detectors.base import BaseDetector

__all__ = [
    "BaseDetector",
]
