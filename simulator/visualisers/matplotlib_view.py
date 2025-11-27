import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from simulator.core import BoidSimulation

def run_mpl_2d(sim: BoidSimulation, interval_ms: int = 30) -> None:
    """
    Simple 2D matplotlib visualiser for BoidSimulation.
    Assumes sim.dim == 2
    """
    if sim.dim != 2:
        raise ValueError("run_mpl_2 only supports 2D simulations.")
    
    fig, ax = plt.subplots(figsize=(8,8))

    # initial scatter
    scat = ax.scatter(sim.pos[:,0], sim.pos[:,1], s= 50)

    ax.set_xlim(0, sim.world_size[0])
    ax.set_ylim(0, sim.world_size[1])
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Boids Simulation (2D)")

    def update(frame):
        sim.step()
        scat.set_offsets(sim.pos)
        return scat,

    _ani = FuncAnimation(fig, update, interval=interval_ms, blit=True)
    plt.show()