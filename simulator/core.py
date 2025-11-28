import numpy as np

from dataclasses import dataclass

@dataclass
class CircleObstacle:
    centre : np.ndarray  # shape (dim,)
    radius : float

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

            align_weight: float = 0.9,
            cohesion_weight: float = 0.2,
            separation_weight: float = 1.2,

            align_radius: float = 2.0,
            cohesion_radius: float = 1.0,
            separation_radius: float = 0.8,

            max_speed: float = 0.05,
            max_force: float = 0.02,

            world_size = (10.0, 10.0, 10.0),

            obstacles = None,
            obstacle_weight: float = 2.0,
            obstacle_influence: float = 0.5,

            cross_species_repulsion: float = 1.0,

            num_predators: int = 0,
            predator_radius: float = 2.5,
            predator_speed_mult: float = 1.5,
            kill_radius: float = 0.3,
            predator_eat: bool = False,

            noise_std: float = 0.02,

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

        self.noise_std = noise_std

        self.rng = rng if rng is not None else np.random.default_rng()

        self.obstacles = obstacles or []
        self.obstacle_weight = obstacle_weight
        self.obstacle_influence = obstacle_influence

        self.cross_species_repulsion = cross_species_repulsion
        self.species = np.zeros(self.n, dtype=int)  # species IDs for each boid

        self.num_predators = num_predators
        self.predator_radius = predator_radius
        self.predator_speed_mult = predator_speed_mult
        self.kill_radius = kill_radius
        self.predator_eat = predator_eat

        # Predator indices + boolean mask
        if num_predators > 0:
            self.predator_ids = self.rng.choice(self.n, size=num_predators, replace=False)
        else:
            self.predator_ids = np.array([], dtype=int)

        self.is_predator = np.zeros(self.n, dtype=bool)
        if self.predator_ids.size > 0:
            self.is_predator[self.predator_ids] = True

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

            si = self.species[i]  # species of this boid

            # Loop over all other boids explicitly 
            for j in range(self.n):
                if j == i:
                    continue  # skip self

                d = dists[j]
                if d <= 0.0:
                    continue

                sj = self.species[j]
                same_species = (si == sj)

                # Same species: full boids rules
                if same_species:
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
                        sep_sum -= offsets[j] / (d**2 + 1e-6)
                        cntS += 1

                # Different species: only repulsion (stronger, slightly longer range) 
                else:
                    cross_sep_radius = self.separation_radius * 1.5
                    if d < cross_sep_radius:
                        sep_sum -= (
                            self.cross_species_repulsion
                            * offsets[j] / (d**2 + 1e-6)
                        )

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
                separation = sep_sum  # might contain only cross-species repulsions

            # Obstacle avoidance
            obstacle_force = np.zeros(self.dim)
            for obs in self.obstacles:
                centre = obs.centre[:self.dim]
                diff = centre - p
                dist = np.linalg.norm(diff)
                if dist < 1e-8:
                    continue

                effective_r = obs.radius + self.obstacle_influence
                if dist < effective_r:
                    # direction away from obstacle centre
                    away = -diff / dist
                    # strength increases as we get closer to the obstacle
                    strength = (effective_r - dist) / effective_r  # in [0, 1]
                    obstacle_force += away * strength

            # Predator / prey dynamics
            predator_force = np.zeros(self.dim)

            is_predator = self.is_predator[i]

            if is_predator:
                # Predator: chase nearest prey within radius
                if self.num_predators < self.n:
                    prey_mask = np.ones(self.n, dtype=bool)
                    prey_mask[self.predator_ids] = False

                    prey_positions = self.pos[prey_mask]
                    if prey_positions.size > 0:
                        diffs = prey_positions - p
                        dists = np.linalg.norm(diffs, axis=1)

                        # Only consider prey within predator_radius
                        within = dists < self.predator_radius
                        if np.any(within):
                            # Vector towards nearest prey
                            nearest_idx = np.argmin(dists)
                            nearest_vec = diffs[nearest_idx]
                            nearest_dist = dists[nearest_idx]

                            if nearest_dist > 1e-8:
                                predator_force += nearest_vec / nearest_dist

                            # ---- OPTIONAL EATING ----
                            if self.predator_eat and nearest_dist < self.kill_radius:
                                # Map back to global index of prey
                                global_preys = np.where(prey_mask)[0]
                                prey_index = global_preys[nearest_idx]

                                # "Eat" prey: respawn somewhere random
                                self.pos[prey_index] = (
                                    self.rng.random(self.dim) * self.world_size
                                )
                                # Give it a random small velocity
                                v0 = self.rng.normal(size=self.dim)
                                v0 /= (np.linalg.norm(v0) + 1e-8)
                                self.vel[prey_index] = v0 * (0.5 * self.max_speed)

            else:
                # Prey: flee from each predator within radius
                for pid in getattr(self, "predator_ids", []):
                    diff = self.pos[pid] - p
                    d = np.linalg.norm(diff)
                    if d < self.predator_radius and d > 1e-8:
                        flee = -diff / d
                        predator_force += 2.0 * flee   # strong flee
            
            # Combine with weights
            steer = (
                self.align_weight * align +
                self.cohesion_weight * cohesion +
                self.separation_weight * separation +
                self.obstacle_weight * obstacle_force +
                predator_force
            )
            # Small random "wander"
            if self.noise_std > 0:
                steer += self.rng.normal(scale=self.noise_std, size=self.dim)

            # Limit steering + speed
            steer = self._limit_vec(steer, self.max_force)
            new_v = self._limit_vec(v + steer, self.max_speed)

            if is_predator:
                new_v = self._limit_vec(new_v, self.max_speed * self.predator_speed_mult)

            new_vel[i] = new_v

        # Update velocities and positions
        self.vel = new_vel
        self.pos = self.pos + self.vel

        # Wrap around world boundaries
        self.pos = np.mod(self.pos, self.world_size)