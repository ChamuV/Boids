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
│   ├── init.py
│   ├── core.py                  # Core BoidSimulation class (state + rules)
│   ├── utils.py                 # Vector helpers, limit(), geometry utilities
│   │
│   ├── visualisers/
│   │   ├── matplotlib_view.py   # Simple 2D scatter animation in matplotlib
│   │   ├── pygame_view.py       # Realtime interactive Pygame renderer
│   │   └── opengl_view.py       # (Optional) High-performance OpenGL renderer
│   │
│   ├── experiments/
│   │   ├── baseline.py                  # Basic flocking demo
│   │   ├── alignment_test.py            # Vary alignment strengths
│   │   ├── separation_test.py           # Visualise separation behaviour
│   │   ├── neighbourhood_radius_test.py # Explore radii sensitivity
│   │   ├── speed_vs_force_test.py       # Stability analysis
│   │
│   └── notebooks/
│       ├── boids_intro.ipynb            # Explanation & interactive plots
│       ├── flocking_metrics.ipynb       # Quantitative flock analysis
│
└── assets/
├── demo.gif                         # Animations for README
└── screenshots/                     # PNG/JPEG images
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
python -m simulator.experiments.baseline
```

### 3. Run another experiment

```bash
python -m simulator.experiments.alignment_test
```

### 4. Try the pygame renderer

```bash
python -m simulator.visualisers.pygame_view 
```
