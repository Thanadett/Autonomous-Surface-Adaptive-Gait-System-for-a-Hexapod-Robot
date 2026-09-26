"""Shared descriptive statistics for every experiment (E1-E7).

One implementation so that all result tables in chapter 4 are computed the
same way: sample standard deviation (ddof=1), percentiles by linear
interpolation, success counts reported as "k/n (p%)".
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Summary:
    n: int
    mean: float
    sd: float
    median: float
    p95: float
    max: float
    min: float

    def mean_sd(self, scale: float = 1.0, digits: int = 3) -> str:
        """'mean ± sd' in the requested unit (scale multiplies the raw values)."""
        if self.n == 0:
            return "n/a"
        return f"{self.mean * scale:.{digits}f} ± {self.sd * scale:.{digits}f}"


def summarize(values: Iterable[float]) -> Summary:
    data = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if data.size == 0:
        nan = float("nan")
        return Summary(0, nan, nan, nan, nan, nan, nan)
    sd = float(np.std(data, ddof=1)) if data.size > 1 else 0.0
    return Summary(
        n=int(data.size),
        mean=float(np.mean(data)),
        sd=sd,
        median=float(np.median(data)),
        p95=float(np.percentile(data, 95)),
        max=float(np.max(data)),
        min=float(np.min(data)),
    )


def success_rate(successes: int, trials: int) -> str:
    """Format as the test plan requires, e.g. '9/10 (90%)'."""
    if trials <= 0:
        return "0/0 (n/a)"
    return f"{successes}/{trials} ({100.0 * successes / trials:.0f}%)"
