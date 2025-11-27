from simulator.core import BoidSimulation
from simulator.visualisers.matplotlib_view import run_mpl_2d

def main(
        N: int = 50,
        align: float = 1.0,
        cohesion: float = 1.0,
        separation: float = 1.5,
) -> None:
    """
    Baseline flocking demo with configurable parameters.
    This is called from the CLI
    """
    sim = BoidSimulation(
        n_boids=N,
        dim=2,
        align_weight=align,
        cohesion_weight=cohesion,
        separation_weight=separation,
    )

    run_mpl_2d(sim)