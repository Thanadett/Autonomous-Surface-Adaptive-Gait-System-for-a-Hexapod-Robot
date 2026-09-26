import math

from hexapod_evaluation.stats import success_rate, summarize


def test_summarize_uses_sample_sd_and_ignores_non_finite() -> None:
    s = summarize([1.0, 2.0, 3.0, 4.0, float("nan"), None])
    assert s.n == 4
    assert math.isclose(s.mean, 2.5)
    assert math.isclose(s.sd, 1.2909944487358056)  # ddof=1
    assert s.max == 4.0 and s.min == 1.0
    assert s.mean_sd(scale=1000, digits=1) == "2500.0 ± 1291.0"


def test_summarize_empty_and_single() -> None:
    assert summarize([]).n == 0
    assert summarize([]).mean_sd() == "n/a"
    assert summarize([5.0]).sd == 0.0


def test_success_rate_format() -> None:
    assert success_rate(9, 10) == "9/10 (90%)"
    assert success_rate(0, 0) == "0/0 (n/a)"
