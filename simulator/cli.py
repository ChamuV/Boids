import argparse

from simulator.experiments import baseline

# Define the available experiments
EXPERIMENTS = {
    "baseline": baseline.main,
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

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Look up the experiment function
    exp_fn = EXPERIMENTS.get(args.experiment)
    if exp_fn is None:
        raise ValueError(f"Unknown experiment: {args.experiment!r}")

    # Call the experiment, passing in CLI parameters
    exp_fn(
        N=args.num_boids,
        align=args.align,
        cohesion=args.cohesion,
        separation=args.separation,
    )