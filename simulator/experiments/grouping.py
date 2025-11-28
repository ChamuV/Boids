import numpy as np

from simulator.core import BoidSimulation
from simulator.visualisers.matplotlib_view import run_mpl_2d


def main(
    N: int = 90,
    align: float = 1.0,
    cohesion: float = 0.4,
    separation: float = 1.0,
    num_species: int = 2,
    species_repulsion: float = 1.0,
    **kwargs
) -> None:
    """
    Grouping experiment:
    - Each species flocks with its own kind (alignment/cohesion/separation)
    - Different species repel each other with strength `species_repulsion`
      via cross_species_repulsion in BoidSimulation.
    - Initial positions are fully random; species are randomly mixed.
    """
    world_w, world_h = 10.0, 10.0
    dim = 2

    # Clamp species count to [1, N]
    num_species = max(1, min(num_species, N))

    sim = BoidSimulation(
        n_boids=N,
        dim=dim,
        align_weight=align,
        cohesion_weight=cohesion,
        separation_weight=separation,
        world_size=(world_w, world_h),
        cross_species_repulsion=species_repulsion,
    )

    rng = sim.rng

    # Assign species labels 0..num_species-1 roughly evenly, but mixed
    idx = np.arange(sim.n)
    rng.shuffle(idx)

    base = sim.n // num_species
    remainder = sim.n % num_species

    start = 0
    for s in range(num_species):
        count = base + (1 if s < remainder else 0)
        sel = idx[start:start + count]
        sim.species[sel] = s
        start += count

    run_mpl_2d(sim)