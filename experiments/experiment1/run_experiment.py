"""
Experiment Runner
=================

CLI entry point for running the planned passage experiment.
"""

import argparse
import logging
import sys
from pathlib import Path

from .passage_simulator import PassageSimulator, SimulationConfig
from src.pilots import get_pilot, list_pilots


def setup_logging(verbose: bool = False):
    """Configure logging."""
    level = logging.DEBUG if verbose else logging.INFO
    format_str = '%(levelname)s: %(message)s' if not verbose else \
                 '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    
    logging.basicConfig(
        level=level,
        format=format_str,
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    
    # Reduce noise from third-party libraries
    logging.getLogger('cfgrib').setLevel(logging.WARNING)
    logging.getLogger('xarray').setLevel(logging.WARNING)
    

def main():
    """Main entry point for experiment runner."""
    parser = argparse.ArgumentParser(
        description='Run planned passage following experiment',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with PD pilot
  uv run python -m experiments.experiment1.run_experiment \\
    --route data/experiment1/route/wr_route_1_20260125_005338.csv \\
    --pilot pd

  # Run with PID pilot and custom gains
  uv run python -m experiments.experiment1.run_experiment \\
    --route data/experiment1/route/wr_route_1_20260125_005338.csv \\
    --pilot pid --ki 0.2 --kp 1.8

  # Run with baseline controller
  uv run python -m experiments.experiment1.run_experiment \\
    --route data/experiment1/route/wr_route_1_20260125_005338.csv \\
    --baseline

  # Run with ML model and GRIB data
  uv run python -m experiments.experiment1.run_experiment \\
    --route data/experiment1/route/wr_route_1_20260125_005338.csv \\
    --gribs data/experiment1/gribs/ \\
    --model models/autopilot_best.keras \\
    --output results/experiment1/

  # Quick test with verbose output
  uv run python -m experiments.experiment1.run_experiment \\
    --route data/experiment1/route/wr_route_1_20260125_005338.csv \\
    --baseline --verbose --max-hours 1
"""
    )
    
    # Required arguments
    parser.add_argument(
        '--route', '-r',
        type=str,
        required=True,
        help='Path to route file (.csv or .kml)'
    )
    
    # Optional arguments
    parser.add_argument(
        '--gribs', '-g',
        type=str,
        default=None,
        help='Path to directory containing GRIB files'
    )
    
    parser.add_argument(
        '--model', '-m',
        type=str,
        default=None,
        help='Path to trained autopilot model (.onnx)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='results/experiment1',
        help='Output directory for results (default: results/experiment1)'
    )
    
    parser.add_argument(
        '--baseline', '-b',
        action='store_true',
        help='Use baseline helm controller instead of ML model'
    )
    
    parser.add_argument(
        '--mock',
        action='store_true',
        help='Use mock autopilot (simple P control) for hypothesis testing'
    )

    # Pilot controller options
    parser.add_argument(
        '--pilot',
        type=str,
        default=None,
        help=f'Use a pilot controller (available: {", ".join(list_pilots())})'
    )
    parser.add_argument('--kp', type=float, default=None, help='Override pilot kp gain')
    parser.add_argument('--kd', type=float, default=None, help='Override pilot kd gain')
    parser.add_argument('--ki', type=float, default=None, help='Override pilot ki gain (PID only)')
    parser.add_argument('--integrator-limit', type=float, default=None,
                        help='Override PID integrator limit (degrees)')
    parser.add_argument('--max-rudder', type=float, default=None,
                        help='Override pilot max rudder (normalised, 1.0 = 25°)')

    parser.add_argument(
        '--max-hours',
        type=float,
        default=24.0,
        help='Maximum simulation duration in hours (default: 24)'
    )
    
    parser.add_argument(
        '--dt',
        type=float,
        default=0.1,
        help='Simulation time step in seconds (default: 0.1)'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)
    
    # Validate inputs
    route_path = Path(args.route)
    if not route_path.exists():
        logger.error(f"Route file not found: {route_path}")
        sys.exit(1)
        
    if args.gribs:
        grib_path = Path(args.gribs)
        if not grib_path.exists():
            logger.error(f"GRIB directory not found: {grib_path}")
            sys.exit(1)
            
    if args.model and not args.baseline:
        model_path = Path(args.model)
        if not model_path.exists():
            logger.warning(f"Model file not found: {model_path}. Will use baseline.")
            args.baseline = True
            
    # Build pilot if requested
    pilot = None
    if args.pilot:
        pilot = get_pilot(args.pilot)
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

    # Create configuration
    config = SimulationConfig(
        dt=args.dt,
        max_duration_hours=args.max_hours,
        use_baseline=args.baseline,
        use_mock=args.mock,
        model_path=args.model,
        verbose=args.verbose,
        pilot=pilot,
    )
    
    # Print configuration
    logger.info("=" * 60)
    logger.info("PLANNED PASSAGE EXPERIMENT")
    logger.info("=" * 60)
    logger.info(f"Route: {args.route}")
    logger.info(f"GRIB data: {args.gribs or 'None (using route predictions)'}")
    if args.pilot:
        pilot_config = {k: getattr(pilot, k) for k in
                        ["kp", "kd", "ki", "max_rudder", "integrator_limit", "dt"]
                        if hasattr(pilot, k)}
        controller_str = f"Pilot: {pilot.name} {pilot_config}"
    elif args.baseline:
        controller_str = "Baseline helm controller"
    elif args.mock:
        controller_str = "Mock autopilot (P control)"
    else:
        controller_str = f"ML model ({args.model})"
    logger.info(f"Controller: {controller_str}")
    logger.info(f"Output: {args.output}")
    logger.info(f"Max duration: {args.max_hours} hours")
    logger.info("=" * 60)
    
    try:
        # Create simulator
        simulator = PassageSimulator(
            route_file=str(route_path),
            grib_dir=args.gribs,
            config=config,
        )
        
        # Run simulation
        results = simulator.run()
        
        # Save results
        output_path = Path(args.output)
        output_path.mkdir(parents=True, exist_ok=True)
        simulator.save_results(str(output_path))
        
        logger.info(f"\nResults saved to {output_path}")
        
        # Return appropriate exit code
        if results['arrived']:
            logger.info("Experiment completed successfully!")
            sys.exit(0)
        else:
            logger.warning("Experiment did not complete (timeout or interrupted)")
            sys.exit(1)
            
    except Exception as e:
        logger.error(f"Experiment failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(2)


if __name__ == '__main__':
    main()
