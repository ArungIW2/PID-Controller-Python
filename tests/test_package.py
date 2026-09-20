"""Smoke tests for the Phase 1 package foundation."""

import pid_controller


def test_package_exposes_semantic_version() -> None:
    """The installed package should expose its current semantic version."""
    assert pid_controller.__version__ == "0.1.0"

