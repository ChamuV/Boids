import pygame
import numpy as np
from simulator.core import BoidSimulation, CircleObstacle

pygame.init()

# --- COLORS ---
WHITE = (240, 240, 240)
GREY  = (180, 180, 180)
BLACK = (20, 20, 20)

# predator color
PRED_COLOR = (200, 60, 60)

# species colours (same order as matplotlib)
SPECIES_COLORS = [
    (8, 48, 107),
    (251, 106, 74),
    (65, 171, 93),
    (128, 125, 186),
    (255, 217, 47),
    (31, 120, 180)
]

# Sandbox to test boid simulation with pygame
def run_pygame_sandbox(
    width=900,
    height=900,
    world_w=10.0,
    world_h=10.0,
):
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Boids Sandbox")

    clock = pygame.time.Clock()

    # scale: world units → pixels
    scale_x = width / world_w
    scale_y = height / world_h

    # Create the simulation
    sim = BoidSimulation(
        n_boids=60,
        dim=2,
        world_size=(world_w, world_h),
        separation_weight=1.2,
        cohesion_weight=0.4,
        align_weight=0.8,
        obstacle_influence=0.7,
    )

    paused = False
    active_species = 0

    running = True
    while running:

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Click to place obstacle
            if event.type == pygame.MOUSEBUTTONDOWN:
                x_pix, y_pix = pygame.mouse.get_pos()
                x = x_pix / scale_x
                y = (height - y_pix) / scale_y

                # left click: obstacle
                if event.button == 1 and not pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    sim.obstacles.append(CircleObstacle(
                        centre=np.array([x, y]),
                        radius=0.4
                    ))

                # shift + left click: predator
                if event.button == 1 and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    pid = np.argmin(sim.is_predator == False)
                    sim.is_predator[pid] = True
                    sim.predator_ids = np.where(sim.is_predator)[0]

            # Key events
            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_SPACE:
                    paused = not paused

                # Switch boundary modes
                if event.key == pygame.K_b:
                    modes = ["wrap", "bounce", "avoid"]
                    i = modes.index(sim.boundary_mode)
                    sim.boundary_mode = modes[(i + 1) % 3]

                # Number keys → change active species
                if pygame.K_1 <= event.key <= pygame.K_9:
                    active_species = event.key - pygame.K_1
                    active_species %= len(SPECIES_COLORS)

                # Paint selected boids with species
                if event.key == pygame.K_p:  
                    mx, my = pygame.mouse.get_pos()
                    mx /= scale_x
                    my = (height - my) / scale_y

                    dists = np.linalg.norm(sim.pos - np.array([mx, my]), axis=1)
                    i = np.argmin(dists)
                    sim.species[i] = active_species


        # Update simulation
        if not paused:
            sim.step()

        # Draw
        screen.fill(WHITE)

        # Draw obstacles
        for obs in sim.obstacles:
            x_pix = int(obs.centre[0] * scale_x)
            y_pix = height - int(obs.centre[1] * scale_y)
            pygame.draw.circle(screen, GREY, (x_pix, y_pix), int(obs.radius * scale_x))

        # Draw boids
        for i in range(sim.n):
            x_pix = int(sim.pos[i, 0] * scale_x)
            y_pix = height - int(sim.pos[i, 1] * scale_y)

            is_pred = sim.is_predator[i]

            if is_pred:
                pygame.draw.circle(screen, PRED_COLOR, (x_pix, y_pix), 8)
            else:
                species_id = int(sim.species[i])
                color = SPECIES_COLORS[species_id % len(SPECIES_COLORS)]
                pygame.draw.circle(screen, color, (x_pix, y_pix), 5)

        # HUD text
        font = pygame.font.SysFont("Arial", 18)
        txt = font.render(
            f"SPACE pause | B boundary={sim.boundary_mode} | Active species={active_species+1}",
            True,
            BLACK,
        )
        screen.blit(txt, (10, 10))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()



if __name__ == "__main__":
    run_pygame_sandbox()