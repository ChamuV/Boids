import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

from simulator.core import BoidSimulation

def create_triangle(position, velocity, scale=0.15):
    """
    Create a rotated triangle patch at `position`,
    pointing in the direction of `velocity`.
    """
    # If velocity is zero, keep previous orientation
    vx, vy = velocity
    angle = np.arctan2(vy, vx)

    # Base triangle (pointing right, before rotation)
    triangle = np.array([
        [scale, 0],          # nose
        [-scale*0.6, scale*0.4],  # back top
        [-scale*0.6, -scale*0.4], # back bottom
    ])

    # Rotation matrix
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s,  c]])

    # Rotate + translate
    rotated = triangle @ R.T + position

    return Polygon(rotated, closed=True, facecolor="#08306B", edgecolor="none")


def run_mpl_2d(sim: BoidSimulation, interval_ms: int = 30):
    """
    Render boids as rotated triangles, updated each frame.
    """
    if sim.dim != 2:
        raise ValueError("run_mpl_2d only supports dim=2 simulations")

    fig, ax = plt.subplots(figsize=(6, 6))  # aesthetic background

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

    # Create triangle patches
    patches = []
    for i in range(sim.n):
        tri = create_triangle(sim.pos[i], sim.vel[i])
        ax.add_patch(tri)
        patches.append(tri)

    # Animation update function
    def update(_frame):
        sim.step()
        for i, tri in enumerate(patches):
            vx, vy = sim.vel[i]
            angle = np.arctan2(vy, vx)

            # Base triangle (pointing right)
            scale = 0.15
            base = np.array([
                [scale, 0],
                [-scale*0.6, scale*0.4],
                [-scale*0.6, -scale*0.4],
            ])

            # Rotate
            c, s = np.cos(angle), np.sin(angle)
            R = np.array([[c, -s], [s, c]])
            rotated = base @ R.T + sim.pos[i]

            # Update polygon coordinates
            tri.set_xy(rotated)

        return patches

    ani = FuncAnimation(
        fig,
        update,
        interval=interval_ms,
        blit=False,   
        repeat=True
    )

    plt.show()