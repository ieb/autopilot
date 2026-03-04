"""
Pilots Package
==============

Registry of autopilot controllers (PD, PID, etc.).
"""

from .base import BasePilot

PILOT_REGISTRY: dict[str, type[BasePilot]] = {}


def register(cls: type[BasePilot]) -> type[BasePilot]:
    """Class decorator that registers a pilot by its ``name`` attribute."""
    PILOT_REGISTRY[cls.name] = cls
    return cls


def get_pilot(name: str, **kwargs) -> BasePilot:
    """Instantiate a registered pilot by name.

    Args:
        name: Pilot name (e.g. "pd", "pid").
        **kwargs: Passed to the pilot constructor.

    Raises:
        KeyError: If no pilot is registered under *name*.
    """
    if name not in PILOT_REGISTRY:
        raise KeyError(
            f"Unknown pilot {name!r}. "
            f"Available: {list(PILOT_REGISTRY.keys())}"
        )
    return PILOT_REGISTRY[name](**kwargs)


def list_pilots() -> list[str]:
    """Return names of all registered pilots."""
    return list(PILOT_REGISTRY.keys())


# Register built-in pilots on import.
from .pd_pilot import PDPilot
from .pid_pilot import PIDPilot

register(PDPilot)
register(PIDPilot)
