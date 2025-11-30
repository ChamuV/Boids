import numpy as np

from simulator.core import BoidSimulation
from simulator.visualisers.matplotlib_view import run_mpl_2d


def main(
        N: int = 80,
        align: float = 1.0,
        cohesion: float = 1.0,
        separation: float = 1.5,
        num_species: int = 3,
        species_repulsion: float = 1.0,
        boundary: str = "wrap",
        **kwargs,
) -> None:
    """
    Grouping demo: several species that flock with their own kind
    and repel other species.

    Species IDs are used by:
      - core.BoidSimulation (cross-species repulsion)
      - matplotlib_view (for colouring).
    """

    sim = BoidSimulation(
        n_boids=N,
        dim=2,
        align_weight=align,
        cohesion_weight=cohesion,
        separation_weight=separation,
        cross_species_repulsion=species_repulsion,
        boundary_mode=boundary,
        # speeds, noise, etc. use core defaults
    )

    # Assign random species IDs in [0, num_species-1]
    rng = np.random.default_rng(0)
    sim.species = rng.integers(0, num_species, size=N)

    run_mpl_2d(sim)