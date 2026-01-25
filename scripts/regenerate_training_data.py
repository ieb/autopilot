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

  # Full training dataset (50 hours, 5 runs with domain randomization)
  uv run python scripts/regenerate_training_data.py --output data/simulated --hours 50 --runs 5

  # Specific scenarios only
  uv run python scripts/regenerate_training_data.py --output data/compass --scenarios medium_upwind delivery
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
            cmd = helm.compute_rudder(yacht.state.heading, 90, 90, yacht.state.heading_rate, 0.1)
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
        logger.info("Scenarios: all standard (medium_upwind, downwind_vmg, mixed_coastal, light_air_reaching)")
    
    try:
        files = generate_training_data(
            output_path=args.output,
            duration_hours=args.hours,
            scenarios=args.scenarios,
            num_runs=args.runs,
            randomize=not args.no_randomize,
            seed=args.seed,
        )
        
        logger.info(f"Generated {len(files)} data files:")
        for f in files:
            logger.info(f"  {f}")
            
        # Summary
        total_hours = args.hours * args.runs
        total_samples = int(total_hours * 3600 * 10)  # 10 Hz
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
