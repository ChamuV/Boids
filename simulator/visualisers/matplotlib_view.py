import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

from simulator.core import BoidSimulation


# --------------------------------------------------
# Colour palette for species
# (extend as many as you want)
# --------------------------------------------------
SPECIES_COLORS = [
    "#08306B",   # dark blue
    "#FB6A4A",   # coral red
    "#41AB5D",   # green
    "#807DBA",   # purple
    "#FFD92F",   # yellow
    "#1F78B4",   # light blue
]


def _make_triangle(position, angle, color, scale=0.15):
    """
    Create a rotated triangle patch at `position`,
    pointing in the direction given by `angle`.
    """
    base = np.array([
        [scale, 0.0],                  # nose
        [-scale * 0.6,  scale * 0.4],  # back top
        [-scale * 0.6, -scale * 0.4],  # back bottom
    ])

    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    rotated = base @ R.T + position

    return Polygon(rotated, closed=True, facecolor=color, edgecolor="none")


def run_mpl_2d(sim: BoidSimulation, interval_ms: int = 30):
    """
    Render boids as rotated triangles, updated each frame.
    Triangles' colours show species.
    Angles are smoothed so they don't twitch too much.
    """
    if sim.dim != 2:
        raise ValueError("run_mpl_2d only supports dim=2 simulations")

    fig, ax = plt.subplots(figsize=(6, 6))

    ax.set_xlim(0, sim.world_size[0])
    ax.set_ylim(0, sim.world_size[1])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)

    # Draw obstacles
    for obs in getattr(sim, "obstacles", []):
        circ = plt.Circle(
            (obs.centre[0], obs.centre[1]),
            obs.radius,
            facecolor="#f0efe9",
            edgecolor="#999999",
            linewidth=1.0,
            alpha=1.0,
        )
        ax.add_patch(circ)

    # initial angles from velocities
    angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])
    smooth_alpha = 0.1  # smoothing factor

    # --------------------------------------------------
    # Create triangle patches WITH species colours
    # --------------------------------------------------
    patches = []
    for i in range(sim.n):
        species_id = int(sim.species[i])
        color = SPECIES_COLORS[species_id % len(SPECIES_COLORS)]

        tri = _make_triangle(sim.pos[i], angles[i], color=color)
        ax.add_patch(tri)
        patches.append(tri)

    # --------------------------------------------------
    # Animation update function
    # --------------------------------------------------
    def update(_frame):
        nonlocal angles

        # advance simulation
        sim.step()

        # new angles from updated velocities
        target_angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])
        angles = (1 - smooth_alpha) * angles + smooth_alpha * target_angles

        # base triangle (unchanged shape)
        scale = 0.15
        base = np.array([
            [scale, 0.0],
            [-scale * 0.6,  scale * 0.4],
            [-scale * 0.6, -scale * 0.4],
        ])

        # update each triangle
        for i, tri in enumerate(patches):
            c, s = np.cos(angles[i]), np.sin(angles[i])
            R = np.array([[c, -s], [s, c]])
            rotated = base @ R.T + sim.pos[i]

            tri.set_xy(rotated)

        return patches

    ani = FuncAnimation(
        fig,
        update,
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    fig._ani = ani
    plt.show()