from simulator.core import BoidSimulation
from simulator.visualisers.matplotlib_view import run_mpl_2d


def main(
    N: int = 100,
    align: float = 1.0,
    cohesion: float = 0.4,
    separation: float = 1.0,
    num_predators: int = 1,
    predator_radius: float = 2.5,
    predator_eat: bool = False,
    boundary: str = "wrap",
    **kwargs,
):
    """
    Predator–prey demo:
      - Some boids are tagged as predators (internal to BoidSimulation).
      - Predators chase prey within predator_radius.
      - If --predator-eat is set, prey are 'eaten' (respawned) when very close.
    """

    sim = BoidSimulation(
        n_boids=N,
        dim=2,
        align_weight=align,
        cohesion_weight=cohesion,
        separation_weight=separation,
        num_predators=num_predators,
        predator_radius=predator_radius,
        predator_speed_mult=1.8,
        kill_radius=0.35,
        predator_eat=predator_eat,
        world_size=(10.0, 10.0),
        boundary_mode=boundary,
        # speeds, noise, etc. from core defaults
    )

    run_mpl_2d(sim)