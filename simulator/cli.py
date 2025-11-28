import argparse

from simulator.experiments import baseline, obstacles

# Define the available experiments
EXPERIMENTS = {
    "baseline": baseline.main,
    "obstacles": obstacles.main,  
}

# Create the argument parser for setting parameters for  simulation
def build_parser():
    parser = argparse.ArgumentParser(
        prog="simulator",
        description="Boids / flocking simulator CLI",
    )

    parser.add_argument(
        "-exp", "--experiment",
        type=str,
        default="baseline",
        choices=EXPERIMENTS.keys(),
        help="Which experiment to run",
    )

    parser.add_argument(
        "-N", "--num-boids",
        type=int,
        default=50,
        help="Number of boids in the simulation",
    )

    parser.add_argument(
        "--align",
        type=float,
        default=1.0,
        help="Alignment weight",
    )

    parser.add_argument(
        "--cohesion",
        type=float,
        default=1.0,
        help="Cohesion weight",
    )

    parser.add_argument(
        "--separation",
        type=float,
        default=1.5,
        help="Separation weight",
    )

    parser.add_argument(
        "--n-obstacles",
        type=int,
        default=3,
        help="Number of obstacles (only for obstacle experiment)",
    )

    parser.add_argument(
    "--obstacles",
    type=str,
    default="",
    help='Obstacle config as "x,y,r;x,y,r;..." (for obstacles experiment).',
    )

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Look up the experiment function
    exp_fn = EXPERIMENTS.get(args.experiment)
    if exp_fn is None:
        raise ValueError(f"Unknown experiment: {args.experiment!r}")

    common_kwargs = dict(
    N=args.num_boids,
    align=args.align,
    cohesion=args.cohesion,
    separation=args.separation,
    )

    # Extra args for the obstacles experiment
    if args.experiment == "obstacles":
        common_kwargs["n_obstacles"] = args.n_obstacles
        common_kwargs["obstacles_str"] = args.obstacles

    exp_fn(**common_kwargs)