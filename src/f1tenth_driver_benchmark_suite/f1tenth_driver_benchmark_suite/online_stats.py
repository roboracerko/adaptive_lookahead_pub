"""
Online statistics calculators for real-time metrics collection.
Supports RMS, max, variance calculations without storing all samples.
"""

import math


class RunningRMS:
    """Running RMS (Root Mean Square) calculator."""
    
    def __init__(self):
        self.n = 0
        self.sum_sq = 0.0
    
    def update(self, x: float):
        """Update with a new sample."""
        self.n += 1
        self.sum_sq += x * x
    
    def value(self) -> float:
        """Get current RMS value."""
        if self.n == 0:
            return float("nan")
        return math.sqrt(self.sum_sq / self.n)
    
    def reset(self):
        """Reset all statistics."""
        self.n = 0
        self.sum_sq = 0.0


class RunningMaxAbs:
    """Running maximum absolute value calculator."""
    
    def __init__(self):
        self.max_abs = 0.0
    
    def update(self, x: float):
        """Update with a new sample."""
        ax = abs(x)
        if ax > self.max_abs:
            self.max_abs = ax
    
    def value(self) -> float:
        """Get current maximum absolute value."""
        return self.max_abs
    
    def reset(self):
        """Reset maximum."""
        self.max_abs = 0.0


class WelfordVar:
    """
    Online variance calculator using Welford's algorithm.
    Numerically stable for large sample sizes.
    """
    
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0  # Sum of squared differences from mean
    
    def update(self, x: float):
        """Update with a new sample."""
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2
    
    def var(self) -> float:
        """Get current variance (sample variance)."""
        if self.n < 2:
            return float("nan")
        return self.M2 / (self.n - 1)
    
    def std(self) -> float:
        """Get current standard deviation."""
        v = self.var()
        if math.isnan(v):
            return float("nan")
        return math.sqrt(v)
    
    def mean_value(self) -> float:
        """Get current mean value."""
        return self.mean
    
    def reset(self):
        """Reset all statistics."""
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0


class RunningMean:
    """Simple running mean calculator."""
    
    def __init__(self):
        self.n = 0
        self.sum = 0.0
    
    def update(self, x: float):
        """Update with a new sample."""
        self.n += 1
        self.sum += x
    
    def value(self) -> float:
        """Get current mean value."""
        if self.n == 0:
            return float("nan")
        return self.sum / self.n
    
    def reset(self):
        """Reset all statistics."""
        self.n = 0
        self.sum = 0.0


class RunningMax:
    """Running maximum value calculator."""
    
    def __init__(self):
        self.max_val = float("-inf")
    
    def update(self, x: float):
        """Update with a new sample."""
        if x > self.max_val:
            self.max_val = x
    
    def value(self) -> float:
        """Get current maximum value."""
        if self.max_val == float("-inf"):
            return float("nan")
        return self.max_val
    
    def reset(self):
        """Reset maximum."""
        self.max_val = float("-inf")


class RunningMin:
    """Running minimum value calculator."""
    
    def __init__(self):
        self.min_val = float("inf")
    
    def update(self, x: float):
        """Update with a new sample."""
        if x < self.min_val:
            self.min_val = x
    
    def value(self) -> float:
        """Get current minimum value."""
        if self.min_val == float("inf"):
            return float("nan")
        return self.min_val
    
    def reset(self):
        """Reset minimum."""
        self.min_val = float("inf")
