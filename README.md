# 🕊️ Boids: A Clean & Modular Flocking Simulation Framework

This repository implements Craig Reynolds’ Boids algorithm in a clean,
modular, research-friendly structure.  
The goal is to make it easy to:

- understand the three core boids rules  
- run experiments (2D / 3D, parameter sweeps, behaviour toggles)  
- visualise flocking with multiple renderers  
- extend the system (obstacle avoidance, predators, etc.)

Everything is organised to keep the **core simulation**, **visualisation**, and **experiments** strictly separate.

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
│   ├── cli.py                   # Command-line interface (select experiment + params)
│   ├── core.py                  # BoidSimulation class (physics + rules + species)
│   ├── main.py                  # Optional entry script (not always needed)
│   │
│   ├── visualisers/
│   │   ├── matplotlib_view.py   # Matplotlib rotated-triangle animation
│   │   └── pygame_view.py       # (Optional) Real-time Pygame renderer
│   │
│   ├── experiments/
│   │   ├── baseline.py                  # Standard flocking demo
│   │   ├── grouping.py                  # Multi-species flock segregation
│   │   ├── obstacles.py                 # Obstacle avoidance behaviour
│   │   ├── neighbourhood_radius_test.py # Explore influence of radii
│   │   ├── speed_vs_force_test.py       # Stability / parameter sweep
│   │
│   └── notebooks/
│       ├── boids_intro.ipynb            # Intro explanation + demos
│       ├── flocking_metrics.ipynb       # Analysis, clustering, diagnostics
│
└── assets/
    ├── demo.gif                         # Demo animations for README
    └── screenshots/                     # Saved PNG/JPEG images
```

---

## 🧠 The Three Core Boids Rules

### 1. Alignment  
Boids steer toward the average heading of nearby boids.

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

### 3. Run grouping (multi-species)

```bash
python -m simulator -exp grouping --num-species 3 --species-repulsion 2.0
```

### 4. Run obstacle-avoidance

```bash
python -m simulator -exp obstacles
```

### 5. Try the pygame renderer

```bash
python -m simulator.visualisers.pygame_view 
```
