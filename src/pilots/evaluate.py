"""
Pilot Evaluator
===============

Closed-loop scenario runner for pilot controllers.
Generalises the CL validation from train_imitation.py to accept
any ``steer_fn(features_22) -> float[-1,1]`` interface.

CLI usage::

    uv run python -m src.pilots.evaluate --pilot pd
    uv run python -m src.pilots.evaluate --pilot pid --ki 0.2
    uv run python -m src.pilots.evaluate --list
"""

import argparse
import sys
from typing import Callable, Optional

import numpy as np

from . import get_pilot, list_pilots, BasePilot


# Re-use scenario definitions from training — single source of truth.
_CL_SCENARIOS = [
    # (name, initial_heading, twd, tws, mode, target_heading, target_awa, target_twa, run_sec, accept_deg)
    ("compass_small",  90.0, 180.0, 12.0, "compass",  95.0,   0.0,   0.0, 120.0,  5.0),
    ("compass_large",  45.0, 180.0, 12.0, "compass", 135.0,   0.0,   0.0,  60.0, 10.0),
    # Wind from 180°, heading 225° → AWA ≈ -45° (port). Start 10° off.
    ("wind_awa_hold", 215.0, 180.0, 15.0, "wind_awa", 225.0, -45.0,  0.0, 120.0, 10.0),
]


def run_cl_scenario(
    steer_fn: Callable[[np.ndarray], float],
    name: str,
    initial_heading: float,
    twd: float,
    tws: float,
    mode: str,
    target_heading: float,
    target_awa: float,
    target_twa: float,
    run_sec: float,
    acceptable_error_deg: float,
    physics_dt: float = 0.1,
    inference_hz: float = 2.0,
    verbose: bool = False,
) -> dict:
    """Run a single closed-loop steering scenario.

    Args:
        steer_fn: callable(np.ndarray[22]) -> float in [-1, 1].
            Receives a single feature vector and returns the full
            rudder command (no blending).
        physics_dt: Physics timestep in seconds (default 0.1 = 10 Hz).
        inference_hz: Rate at which features are computed and the
            pilot is queried (should match training data rate).
        verbose: Print diagnostic snapshots at key timesteps.

    Returns:
        Dict with keys: name, passed, mean_ss_error_deg, max_ss_error_deg,
        max_transient_error_deg, mean_rudder_deg, acceptable_deg.
    """
    from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig
    from src.simulation.wind_model import WindModel, WindConfig
    from src.simulation.wave_model import WaveModel, WaveConfig
    from src.training.data_loader import compute_features, _compute_true_wind

    yacht = YachtDynamics(YachtConfig())
    yacht.state.heading = initial_heading
    yacht.state.stw = 6.0
    yacht.state.sog = 6.0
    wind = WindModel(WindConfig(
        base_twd=twd,
        base_tws_min=tws, base_tws_max=tws,
        shift_rate=0.0, oscillation_enabled=False,
    ))
    wind.reset()
    wind_state = wind.step(0)
    yacht.set_wind(wind_state.twd, wind_state.tws)

    waves = WaveModel(WaveConfig())
    waves.reset()

    tracking_errors: list[float] = []
    rudder_cmds: list[float] = []
    diagnostics: list[tuple] = []

    # Rate tracking for derived features
    prev_awa = yacht.state.awa
    prev_rudder = yacht.state.rudder_angle
    prev_roll = yacht.state.roll

    steps_per_inference = max(1, round(1.0 / (inference_hz * physics_dt)))
    inference_dt = steps_per_inference * physics_dt
    n_steps = int(run_sec / physics_dt)
    rudder_deg = 0.0

    for step_i in range(n_steps):
        wind_state = wind.step(physics_dt)
        yacht.set_wind(wind_state.twd, wind_state.tws)

        # Step wave model
        twa = wind_state.twd - yacht.state.heading
        while twa > 180:
            twa -= 360
        while twa < -180:
            twa += 360
        wave_roll, wave_pitch, wave_yaw = waves.step(
            physics_dt, tws=wind_state.tws,
            heading=yacht.state.heading, twa=twa,
            heel=yacht.state.roll,
        )

        yacht.step(rudder_deg, physics_dt, wave_yaw=wave_yaw)
        total_roll = yacht.state.roll + wave_roll
        total_pitch = yacht.state.pitch + wave_pitch

        # Compute features and query pilot at inference_hz
        if step_i % steps_per_inference == 0:
            awa_rate = (yacht.state.awa - prev_awa) / inference_dt
            if awa_rate > 180 / inference_dt:
                awa_rate -= 360 / inference_dt
            elif awa_rate < -180 / inference_dt:
                awa_rate += 360 / inference_dt
            rudder_vel = (yacht.state.rudder_angle - prev_rudder) / inference_dt
            r_rate = (total_roll - prev_roll) / inference_dt
            prev_awa = yacht.state.awa
            prev_rudder = yacht.state.rudder_angle
            prev_roll = total_roll

            features = compute_features(
                heading=yacht.state.heading,
                pitch=total_pitch,
                roll=total_roll,
                yaw_rate=yacht.state.heading_rate,
                awa=yacht.state.awa,
                aws=yacht.state.aws,
                stw=yacht.state.stw,
                cog=yacht.state.cog,
                sog=yacht.state.sog,
                rudder_angle=yacht.state.rudder_angle,
                target_heading=target_heading,
                target_awa=target_awa,
                target_twa=target_twa,
                mode=mode,
                roll_rate=r_rate,
                awa_rate=awa_rate,
                rudder_velocity=rudder_vel,
                wave_period=waves.state.swell_period,
            )

            # Direct pilot steering — no blending
            rudder_norm = float(np.clip(steer_fn(features), -1.0, 1.0))
            rudder_deg = rudder_norm * 25.0
            rudder_cmds.append(rudder_deg)

            t = step_i * physics_dt
            diagnostics.append((
                t, features[0], features[1], features[19],
                rudder_norm, yacht.state.awa, yacht.state.heading, rudder_deg,
                yacht.state.rudder_angle,
            ))

        # Track error every physics step for accurate measurement
        if mode == "wind_awa":
            err = yacht.state.awa - target_awa
        elif mode == "wind_twa":
            twa_now, _ = _compute_true_wind(
                yacht.state.awa, yacht.state.aws, yacht.state.stw)
            err = twa_now - target_twa
        else:
            err = target_heading - yacht.state.heading
            while err > 180:
                err -= 360
            while err < -180:
                err += 360
        tracking_errors.append(abs(err))

    errors = np.array(tracking_errors)
    ss_start = int(0.75 * len(errors))
    ss_errors = errors[ss_start:]
    mean_ss = float(ss_errors.mean())
    max_ss = float(ss_errors.max())
    passed = mean_ss < acceptable_error_deg

    if verbose and diagnostics:
        print(f"    CL Diagnostics for {name}:")
        print(f"    {'time':>6s}  {'h_err_f':>7s}  {'pd_sug':>7s}  "
              f"{'cmd_deg':>7s}  {'act_deg':>7s}  {'AWA':>7s}  {'heading':>7s}")
        snap_times = [0, 5, 10, 20, 30, 60, 90, run_sec - 1]
        for snap_t in snap_times:
            closest = min(diagnostics, key=lambda d: abs(d[0] - snap_t))
            t, h_err, mode_f, pd_sug, rud_n, awa, hdg, rud_cmd, rud_act = closest
            print(f"    {t:6.1f}  {h_err:+7.3f}  {pd_sug:+7.3f}  "
                  f"{rud_cmd:+7.1f}  {rud_act:+7.1f}  {awa:+7.1f}  {hdg:7.1f}")

    return {
        "name": name,
        "passed": passed,
        "mean_ss_error_deg": round(mean_ss, 2),
        "max_ss_error_deg": round(max_ss, 2),
        "max_transient_error_deg": round(float(errors.max()), 2),
        "mean_rudder_deg": round(float(np.mean(rudder_cmds)), 2),
        "acceptable_deg": acceptable_error_deg,
    }


def run_cl_suite(
    steer_fn: Callable[[np.ndarray], float],
    scenarios: Optional[list] = None,
    verbose: bool = False,
) -> list[dict]:
    """Run the full closed-loop test suite.

    Args:
        steer_fn: callable(np.ndarray[22]) -> float in [-1, 1].
        scenarios: Override scenario list (default: _CL_SCENARIOS).
        verbose: Print diagnostic snapshots.

    Returns:
        List of result dicts.
    """
    scenarios = scenarios or _CL_SCENARIOS
    results = []
    for args in scenarios:
        r = run_cl_scenario(steer_fn, *args, verbose=verbose)
        results.append(r)
    return results


def cl_mean_error(results: list[dict]) -> float:
    """Aggregate score from closed-loop results (lower is better)."""
    return sum(r["mean_ss_error_deg"] for r in results) / max(len(results), 1)


def print_cl_results(results: list[dict]) -> bool:
    """Pretty-print closed-loop validation results.

    Returns:
        True if all scenarios passed.
    """
    all_passed = all(r["passed"] for r in results)
    print("\n" + "=" * 60)
    print("CLOSED-LOOP VALIDATION")
    print("=" * 60)
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        print(f"  [{status}] {r['name']}: "
              f"SS error {r['mean_ss_error_deg']:.1f}\u00b0 "
              f"(max {r['max_ss_error_deg']:.1f}\u00b0, "
              f"accept <{r['acceptable_deg']}\u00b0) "
              f"rudder_mean={r['mean_rudder_deg']:.1f}\u00b0")
    score = cl_mean_error(results)
    print(f"\nCL score: {score:.1f}\u00b0")
    print(f"Overall: {'PASS' if all_passed else 'FAIL'}")
    print("=" * 60 + "\n")
    return all_passed


def evaluate_pilot(
    pilot: BasePilot,
    scenario_name: Optional[str] = None,
    verbose: bool = False,
) -> list[dict]:
    """Evaluate a pilot on the CL test suite.

    Args:
        pilot: Pilot instance.
        scenario_name: Run only a specific scenario (None = all).
        verbose: Print diagnostic snapshots.

    Returns:
        List of result dicts.
    """
    scenarios = _CL_SCENARIOS
    if scenario_name:
        scenarios = [s for s in scenarios if s[0] == scenario_name]
        if not scenarios:
            available = [s[0] for s in _CL_SCENARIOS]
            raise ValueError(
                f"Unknown scenario {scenario_name!r}. "
                f"Available: {available}"
            )

    results = []
    for args in scenarios:
        pilot.reset()
        r = run_cl_scenario(pilot.steer, *args, verbose=verbose)
        results.append(r)
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate autopilot controllers on CL scenarios",
    )
    parser.add_argument("--pilot", type=str, help="Pilot name (e.g. pd, pid)")
    parser.add_argument("--list", action="store_true",
                        help="List available pilots and exit")
    parser.add_argument("--scenario", type=str, default=None,
                        help="Run only a specific scenario")
    parser.add_argument("--verbose", action="store_true",
                        help="Print diagnostic snapshots")

    # Gain overrides
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--ki", type=float, default=None)
    parser.add_argument("--integrator-limit", type=float, default=None)
    parser.add_argument("--max-rudder", type=float, default=None)
    parser.add_argument("--inner", type=str, default="pd",
                        help="Inner pilot for adaptive wrapper (default: pd)")

    args = parser.parse_args()

    if args.list:
        print("Available pilots:")
        for name in list_pilots():
            print(f"  {name}")
        return

    if not args.pilot:
        parser.error("--pilot is required (or use --list)")

    # Build pilot — wrapper types need an inner pilot.
    # AdaptivePilot tunes PD/PID gains so it must wrap a PD/PID directly.
    # SmoothPilot can wrap anything. Valid chains:
    #   pd, pid, adaptive(pd), smooth(pd), smooth(adaptive(pd))
    WRAPPER_TYPES = ("adaptive", "smooth")

    def _build_pilot(name, inner_name):
        if name not in WRAPPER_TYPES:
            return get_pilot(name)

        if name == "adaptive":
            if inner_name in WRAPPER_TYPES:
                parser.error(
                    f"adaptive cannot wrap '{inner_name}' — it needs "
                    f"direct access to PD/PID gains. "
                    f"Try: --pilot smooth --inner adaptive"
                )
            inner = get_pilot(inner_name)
            from .adaptive_pilot import AdaptivePilot
            return AdaptivePilot(inner=inner)

        # smooth — inner can itself be a wrapper
        if inner_name == "adaptive":
            base = get_pilot("pd")
            from .adaptive_pilot import AdaptivePilot
            inner = AdaptivePilot(inner=base)
        elif inner_name in WRAPPER_TYPES:
            inner = _build_pilot(inner_name, "pd")
        else:
            inner = get_pilot(inner_name)
        from .smooth_pilot import SmoothPilot
        return SmoothPilot(inner=inner)

    pilot = _build_pilot(args.pilot, args.inner)

    # Apply gain overrides
    overrides = {}
    if args.kp is not None:
        overrides["kp"] = args.kp
    if args.kd is not None:
        overrides["kd"] = args.kd
    if args.ki is not None:
        overrides["ki"] = args.ki
    if args.integrator_limit is not None:
        overrides["integrator_limit"] = args.integrator_limit
    if args.max_rudder is not None:
        overrides["max_rudder"] = args.max_rudder
    if overrides:
        pilot.configure(**overrides)

    print(f"Evaluating pilot: {pilot.name}")
    config_attrs = {k: getattr(pilot, k) for k in ["kp", "kd", "ki",
                    "max_rudder", "integrator_limit", "dt"]
                    if hasattr(pilot, k)}
    print(f"  Config: {config_attrs}")

    results = evaluate_pilot(pilot, scenario_name=args.scenario,
                             verbose=args.verbose)
    print_cl_results(results)


if __name__ == "__main__":
    main()
