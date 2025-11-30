import pygame
import numpy as np
import pygame_gui

from simulator.core import BoidSimulation, CircleObstacle

pygame.init()

# --- COLORS ---
WHITE = (240, 240, 240)
GREY  = (180, 180, 180)
BLACK = (20, 20, 20)
PRED_COLOR = (200, 60, 60)

SPECIES_COLORS = [
    (8, 48, 107),
    (251, 106, 74),
    (65, 171, 93),
    (128, 125, 186),
    (255, 217, 47),
    (31, 120, 180),
]


def run_pygame_sandbox():
    # --- WINDOW LAYOUT ---
    MAIN_W = 900   # boids world panel
    PANEL_W = 320  # control panel on the right
    HEIGHT  = 900
    WINDOW_SIZE = (MAIN_W + PANEL_W, HEIGHT)

    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Boids Sandbox")

    clock = pygame.time.Clock()
    manager = pygame_gui.UIManager(WINDOW_SIZE)

    # World coordinates (BoidSimulation space)
    world_w, world_h = 10.0, 10.0
    scale_x = MAIN_W / world_w
    scale_y = HEIGHT / world_h

    # --- Create BoidSimulation instance ---
    sim = BoidSimulation(
        n_boids=60,
        dim=2,
        world_size=(world_w, world_h),
        separation_weight=1.2,
        cohesion_weight=0.4,
        align_weight=0.8,
        boundary_mode="wrap",
    )

    paused = False
    active_species = 0
    current_obstacle_radius = 0.4

    # -------------------------------------------
    #           UI CONTROLS (RIGHT PANEL)
    # -------------------------------------------
    panel_rect = pygame.Rect(MAIN_W, 0, PANEL_W, HEIGHT)
    side_panel = pygame_gui.elements.UIPanel(
        relative_rect=panel_rect,
        starting_height=0,
        manager=manager
    )

    y = 20
    x_label = 10
    x_slider = 10
    width_slider = PANEL_W - 20
    row_h = 60

    def add_label(text, y_pos):
        return pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(x_label, y_pos, width_slider, 20),
            text=text,
            container=side_panel,
            manager=manager,
        )

    def add_slider(y_pos, min_val, max_val, start_val):
        return pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect(x_slider, y_pos + 20, width_slider, 20),
            start_value=start_val,
            value_range=(min_val, max_val),
            container=side_panel,
            manager=manager,
        )

    # Alignment
    lbl_align = add_label("Alignment (align_weight)", y)
    slider_align = add_slider(y, 0.0, 2.0, sim.align_weight)
    y += row_h

    # Cohesion
    lbl_cohesion = add_label("Cohesion (cohesion_weight)", y)
    slider_cohesion = add_slider(y, 0.0, 2.0, sim.cohesion_weight)
    y += row_h

    # Separation
    lbl_separation = add_label("Separation (separation_weight)", y)
    slider_separation = add_slider(y, 0.0, 3.0, sim.separation_weight)
    y += row_h

    # Cross-species repulsion
    lbl_species_rep = add_label("Cross-species repulsion", y)
    slider_species_rep = add_slider(y, 0.0, 3.0, sim.cross_species_repulsion)
    y += row_h

    # Predator radius
    lbl_pred_radius = add_label("Predator radius", y)
    slider_pred_radius = add_slider(y, 0.5, 4.0, sim.predator_radius)
    y += row_h

    # Wall influence (for 'avoid')
    lbl_wall_infl = add_label("Wall influence (avoid mode)", y)
    slider_wall_infl = add_slider(y, 0.5, 3.0, sim.wall_influence)
    y += row_h

    # Noise
    lbl_noise = add_label("Noise std (wander)", y)
    slider_noise = add_slider(y, 0.0, 0.1, sim.noise_std)
    y += row_h

    # Predator-eat checkbox
    checkbox_pred_eat = pygame_gui.elements.UISelectionList(
        relative_rect=pygame.Rect(x_slider, y, width_slider, 50),
        item_list=["Predators eat prey"],
        container=side_panel,
        manager=manager,
        allow_multi_select=True,
    )
    y += 60

    # Reset button
    btn_reset = pygame_gui.elements.UIButton(
        relative_rect=pygame.Rect(x_slider, y, width_slider, 30),
        text="Reset simulation",
        container=side_panel,
        manager=manager,
    )
    y += 40

    # Clear obstacles
    btn_clear_obs = pygame_gui.elements.UIButton(
        relative_rect=pygame.Rect(x_slider, y, width_slider, 30),
        text="Clear obstacles",
        container=side_panel,
        manager=manager,
    )
    y += 40

    # Info label (obstacle radius etc.)
    lbl_info = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect(x_label, y, width_slider, 60),
        text="Mouse wheel: change obstacle radius\n"
             "Left-click: obstacle, Shift+Left: predator\n"
             "P near boid: set species, 1–9 choose species\n"
             "SPACE: pause, B: boundary wrap/bounce/avoid",
        container=side_panel,
        manager=manager,
    )

    # -------------------------------------------
    #                 MAIN LOOP
    # -------------------------------------------
    running = True
    while running:
        time_delta = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            # Let pygame_gui see the event first
            manager.process_events(event)

            if event.type == pygame.QUIT:
                running = False

            # Mouse wheel → adjust obstacle radius (global, independent of UI)
            if event.type == pygame.MOUSEWHEEL:
                current_obstacle_radius += 0.05 * event.y
                current_obstacle_radius = max(0.1, min(1.5, current_obstacle_radius))

            # Mouse clicks – only if inside main simulation area
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx < MAIN_W:  # ignore clicks on the right panel
                    x = mx / scale_x
                    y = (HEIGHT - my) / scale_y

                    # Left click: obstacle
                    if event.button == 1 and not pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        sim.obstacles.append(CircleObstacle(
                            centre=np.array([x, y]),
                            radius=current_obstacle_radius,
                        ))

                    # Shift + Left click: predator (promote a non-predator)
                    if event.button == 1 and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        non_preds = np.where(sim.is_predator == False)[0]
                        if len(non_preds) > 0:
                            pid = non_preds[0]
                            sim.is_predator[pid] = True
                            sim.predator_ids = np.where(sim.is_predator)[0]

            # Keyboard (global)
            if event.type == pygame.KEYDOWN:

                # Pause toggle
                if event.key == pygame.K_SPACE:
                    paused = not paused

                # Switch boundary modes
                if event.key == pygame.K_b:
                    modes = ["wrap", "bounce", "avoid"]
                    i = modes.index(sim.boundary_mode)
                    sim.boundary_mode = modes[(i + 1) % 3]

                # Number keys → choose species
                if pygame.K_1 <= event.key <= pygame.K_9:
                    active_species = (event.key - pygame.K_1) % len(SPECIES_COLORS)

                # P → set nearest boid to active_species
                if event.key == pygame.K_p:
                    mx, my = pygame.mouse.get_pos()
                    if mx < MAIN_W:
                        x = mx / scale_x
                        y = (HEIGHT - my) / scale_y
                        dists = np.linalg.norm(sim.pos - np.array([x, y]), axis=1)
                        idx = np.argmin(dists)
                        sim.species[idx] = active_species

            # UI-specific events (buttons, list)
            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == btn_reset:
                    # Rebuild simulation with same boid count, world etc.
                    sim = BoidSimulation(
                        n_boids=sim.n,
                        dim=2,
                        world_size=(world_w, world_h),
                        separation_weight=slider_separation.get_current_value(),
                        cohesion_weight=slider_cohesion.get_current_value(),
                        align_weight=slider_align.get_current_value(),
                        cross_species_repulsion=slider_species_rep.get_current_value(),
                        predator_radius=slider_pred_radius.get_current_value(),
                        wall_influence=slider_wall_infl.get_current_value(),
                        noise_std=slider_noise.get_current_value(),
                        boundary_mode=sim.boundary_mode,
                    )
                    # Reset obstacle radius
                    current_obstacle_radius = 0.4
                    active_species = 0

                if event.ui_element == btn_clear_obs:
                    sim.obstacles = []

            if event.type == pygame_gui.UI_SELECTION_LIST_NEW_SELECTION:
                # handled generically below when reading checkbox state
                pass

        # --- Update sim parameters from sliders / checkbox ---
        sim.align_weight = slider_align.get_current_value()
        sim.cohesion_weight = slider_cohesion.get_current_value()
        sim.separation_weight = slider_separation.get_current_value()
        sim.cross_species_repulsion = slider_species_rep.get_current_value()
        sim.predator_radius = slider_pred_radius.get_current_value()
        sim.wall_influence = slider_wall_infl.get_current_value()
        sim.noise_std = slider_noise.get_current_value()

        # predator_eat from checkbox
        selected = checkbox_pred_eat.get_multi_selection()
        sim.predator_eat = ("Predators eat prey" in selected)

        # Update manager (UI animations, hover, etc.)
        manager.update(time_delta)

        # --- Step the simulation ---
        if not paused:
            sim.step()

        # --- Rendering ---
        screen.fill(WHITE)

        # Clip drawing to main simulation area so we don't draw under the panel
        screen.set_clip(pygame.Rect(0, 0, MAIN_W, HEIGHT))

        # Draw obstacles
        for obs in sim.obstacles:
            ox = int(obs.centre[0] * scale_x)
            oy = HEIGHT - int(obs.centre[1] * scale_y)
            pygame.draw.circle(screen, GREY, (ox, oy), int(obs.radius * scale_x))

        # Draw boids & predators
        for i in range(sim.n):
            x_pix = int(sim.pos[i, 0] * scale_x)
            y_pix = HEIGHT - int(sim.pos[i, 1] * scale_y)

            if sim.is_predator[i]:
                pygame.draw.circle(screen, PRED_COLOR, (x_pix, y_pix), 8)
            else:
                s_id = int(sim.species[i])
                color = SPECIES_COLORS[s_id % len(SPECIES_COLORS)]
                pygame.draw.circle(screen, color, (x_pix, y_pix), 5)

        # Remove clip and draw UI on the right panel
        screen.set_clip(None)
        manager.draw_ui(screen)

        # Small HUD over main area (top-left)
        font = pygame.font.SysFont("Arial", 18)
        hud_text = (
            f"Boundary: {sim.boundary_mode} | "
            f"Active species: {active_species + 1} | "
            f"Obstacle R: {current_obstacle_radius:.2f}"
        )
        text_surface = font.render(hud_text, True, BLACK)
        screen.blit(text_surface, (10, 10))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    run_pygame_sandbox()