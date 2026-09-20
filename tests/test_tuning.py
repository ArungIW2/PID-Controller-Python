"""Tests for tuning and robustness utilities."""

import pytest

from pid_controller import (
    ExperimentScenario,
    FirstOrderProcessConfig,
    PIDConfig,
    ResponseMetrics,
    TuningWeights,
    evaluate_robustness,
    parameter_sweep,
    score_metrics,
    ziegler_nichols_closed_loop,
)
from pid_controller.exceptions import PIDInputError


@pytest.fixture
def scenario() -> ExperimentScenario:
    """Return a small deterministic tuning scenario."""
    return ExperimentScenario(duration=2.0, setpoint=1.0)


def test_score_metrics_uses_explicit_weighted_cost() -> None:
    """Every metric term should contribute predictably to ranking."""
    metrics = ResponseMetrics(1.0, 2.0, 5.0, 1.05, 0.1, 3.0, 4.0)
    weights = TuningWeights(
        iae=1.0,
        ise=2.0,
        overshoot=3.0,
        settling_time=4.0,
        steady_state_error=5.0,
    )
    assert score_metrics(metrics, weights) == pytest.approx(34.5)


def test_missing_metrics_receive_declared_penalty() -> None:
    """Non-settling or undefined overshoot should not rank as free."""
    metrics = ResponseMetrics(None, None, None, 0.0, 0.0, 0.0, 0.0)
    weights = TuningWeights(
        iae=0.0,
        ise=0.0,
        overshoot=1.0,
        settling_time=1.0,
        steady_state_error=0.0,
        missing_metric_penalty=10.0,
    )
    assert score_metrics(metrics, weights) == 20.0


@pytest.mark.parametrize("field", ["iae", "missing_metric_penalty"])
@pytest.mark.parametrize("value", [-1.0, float("inf")])
def test_invalid_tuning_weight_is_rejected(field: str, value: float) -> None:
    """Objective weights must be finite and non-negative."""
    with pytest.raises(PIDInputError):
        TuningWeights(**{field: value})


def test_parameter_sweep_is_sorted_and_deterministic(
    scenario: ExperimentScenario,
) -> None:
    """Candidate ranking should return lowest cost first and repeat exactly."""
    arguments = dict(
        kp_values=(0.5, 1.0),
        ki_values=(0.0, 0.2),
        kd_values=(0.0,),
        base_config=PIDConfig(kp=1.0, sample_time=0.1),
        scenario=scenario,
    )
    first = parameter_sweep(**arguments)
    second = parameter_sweep(**arguments)
    assert len(first) == 4
    assert [item.score for item in first] == sorted(item.score for item in first)
    assert first == second


@pytest.mark.parametrize(
    "overrides",
    [
        {"kp_values": ()},
        {"kp_values": (-1.0,)},
        {"max_candidates": 0},
        {"max_candidates": 1},
    ],
)
def test_invalid_parameter_grid_is_rejected(
    overrides: dict[str, object], scenario: ExperimentScenario
) -> None:
    """Grid tuning must remain bounded and contain valid gains."""
    arguments: dict[str, object] = {
        "kp_values": (1.0,),
        "ki_values": (0.0, 0.1),
        "kd_values": (0.0,),
        "base_config": PIDConfig(kp=1.0, sample_time=0.1),
        "scenario": scenario,
    }
    arguments.update(overrides)
    with pytest.raises(PIDInputError):
        parameter_sweep(**arguments)  # type: ignore[arg-type]


def test_classic_ziegler_nichols_formulas() -> None:
    """P, PI, and PID conversions should match published classic ratios."""
    p = ziegler_nichols_closed_loop(10.0, 4.0, controller_type="P", sample_time=0.1)
    pi = ziegler_nichols_closed_loop(
        10.0, 4.0, controller_type="PI", sample_time=0.1
    )
    pid = ziegler_nichols_closed_loop(
        10.0, 4.0, controller_type="PID", sample_time=0.1
    )
    assert (p.kp, p.ki, p.kd) == (5.0, 0.0, 0.0)
    assert pi.kp == 4.5
    assert pi.ki == pytest.approx(1.35)
    assert (pid.kp, pid.ki, pid.kd) == (6.0, 3.0, 3.0)


@pytest.mark.parametrize(
    ("gain", "period", "kind"),
    [(0.0, 1.0, "P"), (1.0, -1.0, "PI"), (float("inf"), 1.0, "PID"), (1.0, 1.0, "bad")],
)
def test_invalid_ziegler_nichols_inputs_are_rejected(
    gain: float, period: float, kind: str
) -> None:
    """Rule-based conversion requires positive measured inputs and known type."""
    with pytest.raises(PIDInputError):
        ziegler_nichols_closed_loop(
            gain,
            period,
            controller_type=kind,  # type: ignore[arg-type]
            sample_time=0.1,
        )


def test_robustness_evaluates_named_process_variations(
    scenario: ExperimentScenario,
) -> None:
    """One tuning should be testable across process gain and tau changes."""
    runs = evaluate_robustness(
        PIDConfig(kp=1.0, ki=0.2, sample_time=0.1),
        {
            "nominal": scenario,
            "slow": ExperimentScenario(
                duration=2.0,
                setpoint=1.0,
                process=FirstOrderProcessConfig(time_constant=2.0),
            ),
        },
    )
    assert set(runs) == {"nominal", "slow"}
    assert runs["nominal"].simulation != runs["slow"].simulation


def test_empty_robustness_study_is_rejected() -> None:
    """A robustness study needs at least one named variation."""
    with pytest.raises(PIDInputError):
        evaluate_robustness(PIDConfig(kp=1.0), {})

