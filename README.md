# 🕊️ Boids: A Clean & Modular Flocking Simulation Framework

This repository implements Craig Reynolds’ Boids algorithm in a clean,
modular, research-friendly structure.

The goal is to make it easy to:

- understand the three core boids rules  
- run experiments (parameters, boundaries, predators, obstacles, species)  
- visualise flocking with Matplotlib or an interactive Pygame sandbox  
- extend the system (new behaviours, rules, experiments)

Core simulation, visualisation, and experiments are kept **strictly separate**.

---

## 📁 Project Structure

```bash
Boids/
│
├── README.md
├── requirements.txt
│
├── simulator/
│   ├── __init__.py              # Makes 'simulator' a Python package
│   ├── __main__.py              # Allows `python -m simulator`
│   ├── cli.py                   # Command-line interface (experiments + params)
│   ├── core.py                  # BoidSimulation class (physics + rules + species)
│   ├── main.py                  # (Optional) simple entry script
│   │
│   ├── visualisers/
│   │   ├── matplotlib_view.py   # Matplotlib animation
│   │   └── pygame_sandbox.py    # Interactive Pygame "playground" UI
│   │
│   └── experiments/
│       ├── baseline.py                  # Standard flocking demo
│       ├── grouping.py                  # Multi-species flock segregation
│       ├── obstacles.py                 # Obstacle avoidance behaviour
│       ├── predator_chase.py            # Predators chasing prey
│       ├── neighbourhood_radius_test.py # Explore influence of radii
│       └── speed_vs_force_test.py       # Stability / parameter sweep
│
└── assets/
    ├── demo.gif                         # Demo animations for README / site
    └── screenshots/                     # Saved PNG/JPEG images
```

---

## 🧠 The Three Core Boids Rules

### 1. Alignment  
Boids steer toward the average heading (velocity) of nearby boids.

### 2. Cohesion  
Boids steer toward the local centre of mass of neighbours.

### 3. Separation  
Boids steer away when neighbours are too close (collision avoidance).

Together, these create smooth, emergent flocking behaviour even though each rule is simple.

--- 

## 🚀 Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the baseline simulation

```bash
python -m simulator -exp baseline
```

Options you can add, for example:

```bash
python -m simulator -exp baseline \
  -N 80 \
  --align 0.8 \
  --cohesion 0.4 \
  --separation 1.2 \
  --boundary wrap
```

### 3. Run grouping (multi-species)

```bash
python -m simulator -exp grouping --num-species 3 --species-repulsion 2.0
```

### 4. Run obstacle-avoidance

```bash
python -m simulator -exp obstacles
```

### 5. Run predator chase with / without kill

```bash
python -m simulator -exp predator-chase --num-predators 2
python -m simulator -exp predator-chase --num-predators 2 --predator-eat
```

### Boundary modes
Most experiments accept the --boundary flag

```bash
--boundary wrap|bound|avoid
```

---

# ⚙️ Command-Line Interface (CLI) Reference

The simulator includes a flexible CLI for running any experiment with custom parameters.

All experiments are launched using:

```bash
python -m simulator [flags]
```

Summary of all flags:

```bash
-exp, --experiment                 Experiment to run
-N, --num-boids                    Number of boids
--align                            Alignment weight
--cohesion                         Cohesion weight
--separation                       Separation weight

--boundary {wrap,bounce,avoid}     Boundary mode

--num-species                      Multi-species count
--species-repulsion                Cross-species repulsion strength

--n-obstacles                      Number of randomly placed obstacles
--obstacles                        Explicit obstacle list "x,y,r;..."

--num-predators                    Predator count
--predator-radius                  Chase/flee radius
--predator-eat                     Enable predator kill
```

---
🎮 Pygame Sandbox (Interactive Mode)

The interactive sandbox lets you click to place obstacles, drag sliders, and tweak behaviour in real time.

Run it with:

```bash
python -m simulator.visualisers.pygame_sandbox
```

Features:
- Add/delete circular obstacles with the mouse
- Scroll to change obstacle radius
- Real-time sliders for:
- number of boids
- alignment / cohesion / separation
- species count & repulsion
- predator count & chase radius
- Boids rendered as rotating triangles
- Predators rendered as circles
- Instant parameter changes (no restart needed)
