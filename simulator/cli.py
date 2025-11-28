import argparse

from simulator.experiments import baseline, obstacles, grouping, predator_chase

# Define the available experiments
EXPERIMENTS = {
    "baseline": baseline.main,
    "obstacles": obstacles.main,  
    "grouping": grouping.main,
    "predator-chase": predator_chase.main, 
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

    parser.add_argument(
    "--num-species",
    type=int,
    default=2,
    help="Number of species for the 'grouping' experiment.",
    )

    parser.add_argument(
    "--species-repulsion",
    type=float,
    default=1.0,
    help="Extra repulsion strength between different species (grouping experiment).",
    )

    parser.add_argument(
    "--num-predators",
    type=int,
    default=0,
    help="Number of predators in the predator_chase experiment.",
    )

    parser.add_argument(
    "--predator-radius",
    type=float,
    default=2.5,
    help="Chase radius within which predators detect prey.",
    )

    parser.add_argument(
    "--predator-eat",
    action="store_true",
    help="If set, predators can 'eat' prey when they get very close.",
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

    # Extra args for the grouping experiment
    if args.experiment == "grouping":
        common_kwargs["num_species"] = args.num_species
        common_kwargs["species_repulsion"] = args.species_repulsion

    # Extra args for the predator_chase experiment
    if args.experiment == "predator-chase":
        common_kwargs["num_predators"] = args.num_predators
        common_kwargs["predator_radius"] = args.predator_radius
        common_kwargs["predator_eat"] = args.predator_eat

    exp_fn(**common_kwargs)