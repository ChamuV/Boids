import numpy as np

from simulator.core import BoidSimulation, CircleObstacle
from simulator.visualisers.matplotlib_view import run_mpl_2d

def parse_obstacles(obstacles_str: str, world_size):
    """
    Parse a string like "3,5,0.8;7,3,1.0" into a list of CircleObstacle.
    world_size is used only so you can sanity-check later if you like.
    """
    obstacles = []
    if not obstacles_str.strip():
        return obstacles

    parts = obstacles_str.split(";")
    for part in parts:
        if not part.strip():
            continue
        x_str, y_str, r_str = part.split(",")
        x = float(x_str)
        y = float(y_str)
        r = float(r_str)
        obstacles.append(
            CircleObstacle(centre=np.array([x, y]), radius=r)
        )
    return obstacles

def main(
    N: int = 80,
    align: float = 1.0,
    cohesion: float = 0.8,
    separation: float = 1.5,
    n_obstacles: int = 3,
    obstacles_str: str = "",
    **kwargs,
) -> None:
    """
    Obstacle-avoiding flocking demo with configurable parameters.
    Called from the CLI.
    """
    world_w, world_h = 10.0, 10.0

    # If user provided explicit obstacles, use them
    obstacles = parse_obstacles(obstacles_str, (world_w, world_h))
    if not obstacles:
        # Otherwise, create n_obstacles at fixed or random locations
        rng = np.random.default_rng(42)
        for _ in range(n_obstacles):
            centre = np.array([
                rng.uniform(2.0, world_w - 2.0),
                rng.uniform(2.0, world_h - 2.0),
            ])
            radius = rng.uniform(0.5, 1.2)
            obstacles.append(CircleObstacle(centre=centre, radius=radius))

    sim = BoidSimulation( 
        n_boids=N,
        dim=2,
        align_weight=align,
        cohesion_weight=cohesion,
        separation_weight=separation,
        world_size=(world_w, world_h),
        obstacles=obstacles,
        obstacle_weight=2.0,
        obstacle_influence=0.7,
    )

    run_mpl_2d(sim)