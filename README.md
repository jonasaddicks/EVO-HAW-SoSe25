# Drone Swarm: Particle Swarm Optimization

This repository contains an experimental implementation of particle swarm optimization (PSO)
for coordinated multi-drone path planning in a 2D, obstacle-filled environment.
Each drone’s trajectory is represented by a cubic B-spline defined through a sequence of control
points; control points include x, y coordinates plus a local speed parameter. A particle encodes
a full set of control points for all drones, and the PSO searches this joint space to find
collision-free, time- and energy-efficient solutions from a common start to a common goal.

The project was developed as the practical component of an academic assignment in the
“Evolutionary Algorithms” course and focuses on reproducible experiments, configurable
algorithmic parameters (via .env.public), and visual analysis using matplotlib.

The system evaluates candidate solutions based on multiple goals:  
- Avoiding collisions between drones and obstacles,  
- Minimizing overall travel time,  
- Minimizing energy expenditure.  


## Table of Contents

- [Features](#features)
- [Motivation & Use Cases](#motivation)
- [Setup](#setup)
  - [Dependencies](#dependencies)
  - [Configuration](#configuration)
- [Running](#running)
  - [Running a Deterministic Experiment](#deterministic)
- [Algorithmic Details](#algorithm)
- [Visualization](#visualization)
- [Limitations](#limitations)


## <a name="features"></a>Features

## <a name="motivation"></a>Motivation & Use Cases

## <a name="setup"></a>Setup

First, it is recommended to install the application and all dependencies in
a virtual environment:
```
python3 -m venv .venv
source .venv/bin/activate
```

The application is using a `pyproject.toml` and can be installed by running the following
command in the root directory `/EVO-HAW-SoSe25` containing the `pyproject.toml`:
```
pip install .
```
This will install the project like a regular package by building and copying all source
code files into `site-packages/`. All dependencies will be installed automatically. 
If the software is edited the application has to be built again using `pip install .`.

Note: The installation registers a CLI entry point (`droneswarm-pso`) if specified in the
`pyproject.toml` under `[project.scripts]`. The `-e` option installs an editable version,
creating a symbolic link (`.egg-link`) in your environment, allowing local code changes
to take effect immediately without reinstalling.


### <a name="dependencies"></a>Dependencies

The application uses the following libraries which will be installed automatically by
the `pyproject.toml`

```shell
- matplotlib==3.10.7
- networkx==3.4.2
- numpy==2.3.4
- pydantic_settings==2.12.0
- python-dotenv==1.2.1
- scipy==1.16.3
```


### <a name="configuration"></a>Configuration

Customize the `.env.public` file to match your desired runtime parameters:

```dotenv
DEBUG=False
SEED_ENVIRONMENT=13# Seed for the randomizer of the environment generation
SEED_PARTICLE=42# Seed for the randomizer of the particle swarm optimization

# DRONE PARAMETERS
NUMBER_DRONES=3# Number of drones in an environment
DRONE_RADIUS=2.0# Size of a drone
DRONE_MAX_SPEED=5.0# Maximum drone speed
INITIAL_CONTROL_POINTS=4# Number of points in a single drone path from start to goal

# ENVIRONMENT PARAMETERS
ENVIRONMENT_SIZE_X=100# Width of the environment
ENVIRONMENT_SIZE_Y=100# Height of the environment

ENVIRONMENT_TRAVERSABLE=False# Forces at least one path without any collisions from start to goal (NOTE: depending on environment size and number of obstacles, the calculation power needed can be exceedingly high.)
NUMBER_OBSTACLES=8# Number of obstacles in the environment
AVG_SIZE_OBSTACLE=10.0# Average size of all the obstacles

START_X=10# Starting point X-coordinate
START_Y=10# Starting point Y-coordinate
START_RADIUS=5# Starting point radius

GOAL_X=90# Goal point X-coordinate
GOAL_Y=90# Goal point Y-coordinate
GOAL_RADIUS=5# Goal point radius

# PARTICLE SWARM OPTIMIZATION PARAMETERS
PSO_PARTICLES=30# Number of particles to explore the solution space
PSO_ITERATIONS=100# Number of iterations the particle swarm optimization will perform

PSO_MAX_INITIAL_VELOCITY_X=10.0# Max velocity of particle (X) when initializing for the first time
PSO_MAX_INITIAL_VELOCITY_Y=10.0# Max velocity of particle (Y) when initializing for the first time
PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY=1.3# Max drone velocity when initializing for the first time
PSO_INITIAL_POSITION_BOUNDS=30# Area around an initial point when generating in which the actual point generates for the first time
PSO_INITIAL_DISTANCE_PATHS=10# Probable distance between different drone paths when initializing for the first time

PSO_MAX_VELOCITY_X=15.0# Max velocity of a particle (X)
PSO_MAX_VELOCITY_Y=15.0# Max velocity of a particle (Y)
PSO_MAX_VELOCITY_DRONE_VELOCITY=2.0# Max velocity of a particle (drone velocity)
PSO_VELOCITY_DAMPING=0.5# Scalar which a particle is scaled with when violating environment bounds and bouncing back

PSO_FLUSH_SHARE=0.04# Portion of particles to be flushed each generation
PSO_FLUSH_WHEN=0.4# After how many generations will the flush occur for the first time (depending on the max number of iterations)

PSO_DECREASE_MAX_VELOCITY_WHEN=0.5# After how many generations will the max velocity begin to adapt (depending on the max number of iterations)
PSO_DECREASE_MAX_VELOCITY_GOAL=3.0# Max velocity value to gradually be approached through the generations
PSO_DECREASE_INITIAL_VELOCITY_WHEN=0.7# After how many generations will the initial velocity begin to adapt (depending on the max number of iterations)
PSO_DECREASE_INITIAL_VELOCITY_GOAL=1.5# Max initial velocity value to gradually be approached through the generations

PSO_WEIGHT_PERSONAL_POSITION=0.8# Weight the current personal position of a particle
PSO_WEIGHT_PERSONAL_BEST=0.7# Weight the current best personal position of a particle
PSO_WEIGHT_GLOBAL_BEST=0.1# Weight the current best global position of all particles

PSO_INCREASE_WEIGHT_GLOBAL_WHEN=0.8# After how many generations will the global weight begin to adapt (depending on the max number of iterations)
PSO_INCREASE_WEIGHT_GLOBAL_GOAL=0.9# Global weight value to gradually be approached through the generations
PSO_DECREASE_WEIGHT_PERSONAL_WHEN=0.75# After how many generations will the personal weight begin to adapt (depending on the max number of iterations)
PSO_DECREASE_WEIGHT_PERSONAL_GOAL=0.5# Personal weight value to gradually be approached through the generations

FITNESS_WEIGHT_ENERGY=0.5# How important is energy usage
FITNESS_WEIGHT_TIME=0.5# How important is time usage
FITNESS_WEIGHT_COLLISIONS_OBSTACLES=150.0# How important is obstacle collision prevention
FITNESS_WEIGHT_COLLISIONS_DRONES=30.0# How important is drone collision prevention
```
If the `.env.public` cannot be found the application will use default values.


## <a name="running"></a>Running

Use the CLI entry point to launch the application: 
```droneswarm-pso```
This command is registered automatically via the pyproject.toml under `[project.scripts]`.


### <a name="deterministic"></a>Running a Deterministic Experiment

## <a name="algorithm"></a>Algorithmic Details

## <a name="visualization"></a>Visualization

## <a name="limitations"></a>Limitations