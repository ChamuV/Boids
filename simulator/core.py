import numpy as np

class BoidSimulation:
    """
    Core Boids / flocking simulation.

    Holds:
      - positions (self.pos)
      - velocities (self.vel)
      - rule parameters (alignment, cohesion, separation)
      - world size and dimensionality

    Call .step() once per frame to advance the simulation.
    """

    def __init__(
            self,
            n_boids: int = 50,
            dim: int = 2,

            align_weight: float = 1.0,
            cohesion_weight: float = 0.8,
            separation_weight: float = 1.5,

            align_radius: float = 1.0,
            cohesion_radius: float = 1.0,
            separation_radius: float = 0.4,

            max_speed: float = 0.03,
            max_force: float = 0.01,

            world_size = (10.0, 10.0, 10.0),
            rng: np.random.Generator | None = None
        ):
        assert dim in (2, 3), "Only 2D and 3D simulations are supported."
        
        self.n = n_boids
        self.dim = dim

        # Rule weights
        self.align_weight = align_weight
        self.cohesion_weight = cohesion_weight
        self.separation_weight = separation_weight

        # Rule radii
        self.align_radius = align_radius
        self.cohesion_radius = cohesion_radius
        self.separation_radius = separation_radius

        self.max_speed = max_speed
        self.max_force = max_force

        self.rng = rng if rng is not None else np.random.default_rng()

        # World size (tuple of length dim)
        self.world_size = np.array(world_size[:dim])

        # Initialize positions in the world by having a box
        self.pos = self.rng.random((n_boids, dim)) * self.world_size

        # Initialize velocities randomly as small vectors
        angles = self.rng.random(n_boids) * 2 * np.pi
        if dim == 2:
            self.vel = np.stack(
                (np.cos(angles), np.sin(angles)), axis=1
            ) * (0.5 * max_speed)
        else:  # dim == 3
            v = self.rng.normal(size=(n_boids, 3))
            v /= np.linalg.norm(v, axis=1, keepdims=True) + 1e-8
            self.vel = v * (0.5 * max_speed)

    def _limit_vec(self, v: np.ndarray, maxval: float) -> np.ndarray:
        """Return vector v limited to magnitude <= maxval."""
        norm = np.linalg.norm(v)
        if norm > maxval and norm > 0:
            return v * (maxval / norm)
        return v
    
    def step(self) -> None:
        """Advance the simulation by one time step."""
        new_vel = np.zeros_like(self.vel)

        for i in range(self.n):
            p = self.pos[i]
            v = self.vel[i]

            # Offsets and distances to all other boids
            offsets = self.pos - p        # shape (n, dim)
            dists = np.linalg.norm(offsets, axis=1)

            # Accumulators for each rule
            align_sum = np.zeros(self.dim)
            cohesion_sum = np.zeros(self.dim)
            sep_sum = np.zeros(self.dim)

            cntA = 0  # alignment neighbours
            cntC = 0  # cohesion neighbours
            cntS = 0  # separation neighbours

            # Loop over all other boids explicitly 
            for j in range(self.n):
                if j == i:
                    continue  # skip self

                d = dists[j]
                if d <= 0.0:
                    continue

                # Alignment: match average heading
                if d < self.align_radius:
                    align_sum += self.vel[j]
                    cntA += 1

                # Cohesion: move towards centre of mass
                if d < self.cohesion_radius:
                    cohesion_sum += self.pos[j]
                    cntC += 1

                # Separation: repel if too close
                if d < self.separation_radius:
                    # inverse-square weighting for stronger nearby repulsion
                    sep_sum -= offsets[j] / (d**2 + 1e-6)
                    cntS += 1

            # Turn sums into rule vectors 
            if cntA > 0:
                align = align_sum / cntA
            else:
                align = np.zeros(self.dim)

            if cntC > 0:
                centre = cohesion_sum / cntC
                cohesion = centre - p
            else:
                cohesion = np.zeros(self.dim)

            if cntS > 0:
                separation = sep_sum
            else:
                separation = np.zeros(self.dim)

            # Combine with weights
            steer = (
                self.align_weight * align +
                self.cohesion_weight * cohesion +
                self.separation_weight * separation
            )

            # Limit steering + speed
            steer = self._limit_vec(steer, self.max_force)
            new_v = self._limit_vec(v + steer, self.max_speed)
            new_vel[i] = new_v

        # Update velocities and positions
        self.vel = new_vel
        self.pos = self.pos + self.vel

        # Wrap around world boundaries
        self.pos = np.mod(self.pos, self.world_size)