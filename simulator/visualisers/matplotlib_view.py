import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

from simulator.core import BoidSimulation


def _make_triangle(position, angle, scale=0.15):
    """
    Create a rotated triangle patch at `position`,
    pointing in the direction given by `angle` (radians).
    """
    # Base triangle (pointing right before rotation)
    base = np.array([
        [scale, 0.0],               # nose
        [-scale * 0.6,  scale * 0.4],   # back top
        [-scale * 0.6, -scale * 0.4],   # back bottom
    ])

    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])

    rotated = base @ R.T + position
    return Polygon(rotated, closed=True, facecolor="#08306B", edgecolor="none")


def run_mpl_2d(sim: BoidSimulation, interval_ms: int = 30):
    """
    Render boids as rotated triangles, updated each frame.
    Uses a smoothed heading so triangles don't twitch too much.
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

    # Draw obstacles if any
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

    # initial angles from current velocities
    angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])
    smooth_alpha = 0.1  # 0=very smooth/slow, 1=no smoothing

    # Create triangle patches
    patches = []
    for i in range(sim.n):
        tri = _make_triangle(sim.pos[i], angles[i])
        ax.add_patch(tri)
        patches.append(tri)

    # Animation update function
    def update(_frame):
        nonlocal angles

        # advance simulation
        sim.step()

        # target angles from (updated) velocities
        target_angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])

        # simple exponential smoothing of angle
        angles = (1.0 - smooth_alpha) * angles + smooth_alpha * target_angles

        # update triangle geometry
        scale = 0.15
        base = np.array([
            [scale, 0.0],
            [-scale * 0.6,  scale * 0.4],
            [-scale * 0.6, -scale * 0.4],
        ])

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

    # keep a reference just in case
    fig._ani = ani

    plt.show()