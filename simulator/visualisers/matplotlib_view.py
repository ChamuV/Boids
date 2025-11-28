import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Circle

from simulator.core import BoidSimulation

# Colour palette for species
SPECIES_COLORS = [
    "#08306B",   # dark blue
    "#FB6A4A",   # coral red
    "#41AB5D",   # green
    "#807DBA",   # purple
    "#FFD92F",   # yellow
    "#1F78B4",   # light blue
]

PREDATOR_COLOR = "#555555"   # dark grey
PREDATOR_RADIUS = 0.1      # slightly larger size


def _make_triangle(position, angle, color, scale=0.15):
    """Rotated triangle (prey)."""
    base = np.array([
        [scale, 0.0],
        [-scale * 0.6,  scale * 0.4],
        [-scale * 0.6, -scale * 0.4],
    ])

    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    rotated = base @ R.T + position

    return Polygon(rotated, closed=True, facecolor=color, edgecolor="none")


def run_mpl_2d(sim: BoidSimulation, interval_ms: int = 30):
    """
    Render boids:
      - Prey = triangles, coloured by species
      - Predators = grey circles
      - Angles smoothed to avoid twitching
    """
    if sim.dim != 2:
        raise ValueError("run_mpl_2d only supports 2D simulations.")

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, sim.world_size[0])
    ax.set_ylim(0, sim.world_size[1])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)

    # Draw obstacles
    for obs in getattr(sim, "obstacles", []):
        circ = Circle(
            (obs.centre[0], obs.centre[1]),
            obs.radius,
            facecolor="#f0efe9",
            edgecolor="#999999",
            linewidth=1.0,
        )
        ax.add_patch(circ)

    # Extract species and predator flags
    species = getattr(sim, "species", np.zeros(sim.n, dtype=int))
    is_pred = getattr(sim, "is_predator", np.zeros(sim.n, dtype=bool))

    # Initial heading angles
    angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])
    smooth_alpha = 0.1

    # Create patches
    patches = []
    for i in range(sim.n):
        if is_pred[i]:
            # Predator = circle
            circ = Circle(sim.pos[i], PREDATOR_RADIUS,
                          facecolor=PREDATOR_COLOR, edgecolor="none")
            ax.add_patch(circ)
            patches.append(("pred", circ))
        else:
            # Prey = triangle
            col = SPECIES_COLORS[species[i] % len(SPECIES_COLORS)]
            tri = _make_triangle(sim.pos[i], angles[i], col)
            ax.add_patch(tri)
            patches.append(("prey", tri))

    # Animation step
    def update(_frame):
        nonlocal angles

        sim.step()

        # Update angles with smoothing
        target_angles = np.arctan2(sim.vel[:, 1], sim.vel[:, 0])
        angles = (1 - smooth_alpha) * angles + smooth_alpha * target_angles

        # Base triangle shape
        scale = 0.15
        base = np.array([
            [scale, 0.0],
            [-scale * 0.6,  scale * 0.4],
            [-scale * 0.6, -scale * 0.4],
        ])

        # Update shapes
        for i, (kind, patch) in enumerate(patches):

            if kind == "pred":
                # Update predator circle
                patch.center = sim.pos[i]

            else:
                # Update prey triangle
                c, s = np.cos(angles[i]), np.sin(angles[i])
                R = np.array([[c, -s], [s, c]])
                rotated = base @ R.T + sim.pos[i]
                patch.set_xy(rotated)

        return [p[1] for p in patches]

    ani = FuncAnimation(fig, update, interval=interval_ms,
                        blit=False, repeat=True)
    fig._ani = ani

    plt.show()