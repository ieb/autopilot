#!/usr/bin/env python3
"""
Regenerate Training Data
========================

This script regenerates the simulated training data using the fixed helm controller.
Run this after the helm controller bug fix to create correct training data.

Usage:
    uv run python scripts/regenerate_training_data.py --output data/simulated
    uv run python scripts/regenerate_training_data.py --output data/simulated --hours 50 --runs 5

After generating data, retrain the model:
    uv run python -m src.training.train_imitation data/simulated
"""

import argparse
import logging
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.data_generator import generate_training_data

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(
        description='Regenerate simulated training data with fixed helm controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Quick test (1 hour of data)
  uv run python scripts/regenerate_training_data.py --output data/test --hours 1

  # Full training dataset (20 hours, 3 runs with domain randomization)
  uv run python scripts/regenerate_training_data.py --output data/simulated --hours 20 --runs 3

  # Specific scenarios only
  uv run python scripts/regenerate_training_data.py --output data/compass --scenarios medium_upwind delivery

  # Larger perturbations for aggressive error recovery training
  uv run python scripts/regenerate_training_data.py --output data/simulated --hours 20 --runs 3 --perturb-magnitude 60
"""
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='data/simulated',
        help='Output directory for training data (default: data/simulated)'
    )
    
    parser.add_argument(
        '--hours', '-H',
        type=float,
        default=10.0,
        help='Total hours of data per run (default: 10.0)'
    )
    
    parser.add_argument(
        '--runs', '-r',
        type=int,
        default=1,
        help='Number of runs for domain randomization (default: 1)'
    )
    
    parser.add_argument(
        '--scenarios', '-s',
        type=str,
        nargs='+',
        default=None,
        help='Specific scenarios to use (default: all standard scenarios)'
    )
    
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Random seed for reproducibility (default: 42)'
    )
    
    parser.add_argument(
        '--no-randomize',
        action='store_true',
        help='Disable domain randomization'
    )
    
    parser.add_argument(
        '--warmup',
        type=float,
        default=90.0,
        help='Warm-up seconds before recording (skip transient, default: 90)'
    )
    
    parser.add_argument(
        '--json',
        action='store_true',
        help='Write legacy .jsonlog output instead of binary .bin (default: binary)'
    )
    parser.add_argument(
        '--no-perturbations',
        action='store_true',
        help='Disable heading perturbations (default: enabled)'
    )
    parser.add_argument(
        '--perturb-interval',
        type=float,
        default=15.0,
        help='Mean seconds between heading perturbations (default 15)'
    )
    parser.add_argument(
        '--perturb-magnitude',
        type=float,
        default=45.0,
        help='Max perturbation magnitude in degrees (default 45)'
    )
    
    args = parser.parse_args()
    
    # Verify the helm controller fix is in place
    logger.info("Verifying helm controller fix...")
    try:
        from src.simulation.helm_controller import HelmController, HelmConfig, SteeringMode
        from src.simulation.yacht_dynamics import YachtDynamics
        
        helm = HelmController(HelmConfig(reaction_delay=0.0))
        helm.set_mode(SteeringMode.COMPASS, 85.0)
        
        yacht = YachtDynamics()
        yacht.state.heading = 95.0
        yacht.state.stw = 6.0
        
        # Run for 5 seconds
        for _ in range(50):
            cmd = helm.compute_rudder(yacht.state.heading, 90, 90, yacht.state.heading_rate, 0.1, aws=15.0, stw=6.0)
            yacht.step(cmd, 0.1)
        
        final_error = abs(85 - yacht.state.heading)
        if final_error > 5.0:
            logger.error(f"Helm controller test failed! Final error: {final_error:.1f}°")
            logger.error("The helm controller fix may not be properly applied.")
            sys.exit(1)
        else:
            logger.info(f"Helm controller test passed (error: {final_error:.1f}°)")
            
    except Exception as e:
        logger.error(f"Failed to verify helm controller: {e}")
        sys.exit(1)
    
    # Generate data
    logger.info(f"Generating {args.hours} hours of data x {args.runs} runs")
    logger.info(f"Output directory: {args.output}")
    
    if args.scenarios:
        logger.info(f"Scenarios: {args.scenarios}")
    else:
        logger.info("Scenarios: all standard (medium_upwind, downwind_vmg, mixed_coastal, light_air_reaching, error_recovery)")
    
    logger.info(f"Warm-up period: {args.warmup}s (skip initial transient)")
    if not args.no_perturbations:
        logger.info(f"Perturbations: multi-scale (max ±{args.perturb_magnitude:.0f}°) every ~{args.perturb_interval:.0f}s")
    else:
        logger.info("Perturbations: disabled")
    
    try:
        files = generate_training_data(
            output_path=args.output,
            duration_hours=args.hours,
            scenarios=args.scenarios,
            num_runs=args.runs,
            randomize=not args.no_randomize,
            seed=args.seed,
            warmup_seconds=args.warmup,
            enable_perturbations=not args.no_perturbations,
            perturb_interval_sec=args.perturb_interval,
            perturb_magnitude_deg=args.perturb_magnitude,
            output_json=args.json,
        )
        
        logger.info(f"Generated {len(files)} data files:")
        for f in files:
            logger.info(f"  {f}")
            
        # Summary
        total_hours = args.hours * args.runs
        total_samples = int(total_hours * 3600 * 2)  # 2 Hz recording rate
        logger.info("")
        logger.info("=" * 60)
        logger.info("TRAINING DATA GENERATION COMPLETE")
        logger.info("=" * 60)
        logger.info(f"Total duration: {total_hours:.1f} hours")
        logger.info(f"Approximate samples: {total_samples:,}")
        logger.info("")
        logger.info("Next steps:")
        logger.info("  1. Train the model:")
        logger.info(f"     uv run python -m src.training.train_imitation {args.output}")
        logger.info("")
        logger.info("  2. Run the passage experiment:")
        logger.info("     uv run python -m experiments.experiment1.run_experiment \\")
        logger.info("       --route data/experiment1/route/wr_route_1_*.csv \\")
        logger.info("       --model models/autopilot.onnx")
        
    except Exception as e:
        logger.error(f"Failed to generate training data: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
