#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mathematical utility functions.
"""

import math


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]. Alias for wrap_angle."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
