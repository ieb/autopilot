"""
Experiment 1: Planned Passage Following
========================================

Tests the autopilot model's ability to follow a pre-planned sailing route
using real weather data from GRIB files.

Components:
- route_parser: Parse route CSV with waypoints
- grib_reader: Read GRIB weather data files
- wind_provider: Interpolate wind data for position/time
- wave_provider: Interpolate wave data for sea state
- navigator: Waypoint tracking and steering mode selection
- passage_simulator: Main simulation orchestrator
- metrics: Performance tracking and reporting
- run_experiment: CLI entry point
- export_track: Export track to GPX for OpenSeaMap

Usage:
    uv run python -m experiments.experiment1.run_experiment --help
    uv run python -m experiments.experiment1.export_track --help
"""

# Lazy imports to avoid requiring all dependencies at import time
def __getattr__(name):
    """Lazy import of submodules."""
    if name == 'RouteParser':
        from .route_parser import RouteParser
        return RouteParser
    elif name == 'Waypoint':
        from .route_parser import Waypoint
        return Waypoint
    elif name == 'GribReader':
        from .grib_reader import GribReader
        return GribReader
    elif name == 'WindProvider':
        from .wind_provider import WindProvider
        return WindProvider
    elif name == 'WaveProvider':
        from .wave_provider import WaveProvider
        return WaveProvider
    elif name == 'Navigator':
        from .navigator import Navigator
        return Navigator
    elif name == 'SteeringMode':
        from .navigator import SteeringMode
        return SteeringMode
    elif name == 'PassageSimulator':
        from .passage_simulator import PassageSimulator
        return PassageSimulator
    elif name == 'MetricsTracker':
        from .metrics import MetricsTracker
        return MetricsTracker
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    'RouteParser',
    'Waypoint',
    'GribReader',
    'WindProvider',
    'WaveProvider',
    'Navigator',
    'SteeringMode',
    'PassageSimulator',
    'MetricsTracker',
]
