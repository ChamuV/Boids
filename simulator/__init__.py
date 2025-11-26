"""
simulator package
A clean, modular framework for Boids / flocking simulations.
"""

from .core import BoidSimulation
from . import visualisers
from . import experiments

__all__ = [
    "BoidSimulation",
    "visualisers",
    "experiments",
]

__version__ = "0.1.0" 