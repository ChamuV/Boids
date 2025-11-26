import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Boids Parameters
N = 50  # Number of boids
width, height = 10, 10  # Size of the environment
max_speed = 0.03  # Maximum speed of boids
max_force = 0.01  # Maximum steering force

align_radius = 1.0
cohesion_radius = 1.0
separation_radius = 0.4

# Initialize boid positions and velocities
pos = np.random.rand(N, 2) * [width, height]
vel = (np.random.rand(N, 2) - 0.5) * max_speed

# Boid behavior functions
def limit(v, maxval):
    mag = np.linalg.norm(v)
    if mag > maxval:
        return v / mag * maxval
    return v

def boid_step():
    global pos, vel
    new_vel = np.zeros_like(vel)

    for i in range(N):
        p, v = pos[i], vel[i]

        # Neighbourhood masks
        offsets = pos - p
        dists = np.linalg.norm(offsets, axis=1)

        # Alignment
        mask_align = (dists < align_radius) & (dists > 0)
        align_vec = np.mean(vel[mask_align], axis=0) if mask_align.any() else np.zeros(2)

        # Cohesion
        mask_cohesion = (dists < cohesion_radius) & (dists > 0)
        if mask_cohesion.any():
            centre = np.mean(pos[mask_cohesion], axis=0)
            cohesion_vec = centre - p
        else:
            cohesion_vec = np.zeros(2)

        # Separation
        mask_sep = (dists < separation_radius) & (dists > 0)
        if mask_sep.any():
            sep_vec = -np.sum(offsets[mask_sep] / dists[mask_sep][:, None], axis=0)
        else:
            sep_vec = np.zeros(2)

        # Combine weighted
        steer = (1.0 * align_vec +
                 0.8 * cohesion_vec +
                 1.5 * sep_vec)

        steer = limit(steer, max_force)
        new_vel[i] = limit(v + steer, max_speed)

    vel[:] = new_vel

    # update position
    pos[:] += vel

    # wrap around edges
    pos[:, 0] %= width
    pos[:, 1] %= height

# Visualization
fig, ax = plt.subplots(figsize=(6, 6))
scat = ax.scatter(pos[:, 0], pos[:, 1], s=20, color="royalblue")

ax.set_xlim(0, width)
ax.set_ylim(0, height)
ax.set_aspect('equal')


def update(frame):
    boid_step()
    scat.set_offsets(pos)
    return scat,


ani = FuncAnimation(fig, update, interval=30)
plt.show()