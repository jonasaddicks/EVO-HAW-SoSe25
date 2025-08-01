from copy import deepcopy
from typing import List, Callable

from DroneSwarmPathOpti.simulation import Environment

from .particle import Particle, DronePath


class PSO:

    fitness_function: Callable[[list[DronePath], Environment], float]
    environment: Environment
    num_particles: int
    max_iterations: int

    particles: List[Particle]

    global_best_position: list[DronePath]
    global_best_fitness: float

    def __init__(self, fitness_function, environment: Environment, num_particles: int=30, max_iterations: int=100):
        self.fitness_function = fitness_function
        self.environment = environment
        self.num_particles = num_particles
        self.max_iterations = max_iterations

        self.particles = [Particle() for _ in range(self.num_particles)]

        self.global_best_position = deepcopy(self.particles[0].particle_position)
        self.global_best_fitness = float("inf")

    def optimize(self):
        for iteration in range(self.max_iterations):
            for particle in self.particles:
                fitness = self.fitness_function(particle.particle_position, self.environment) # Calculate fitness for current particle

                particle.current_fitness = fitness

                # Update personal best
                if fitness < particle.best_fitness:
                    particle.best_fitness = fitness
                    particle.best_position = deepcopy(particle.particle_position)

                # Update global best
                if fitness < self.global_best_fitness:
                    self.global_best_fitness = fitness
                    self.global_best_position = deepcopy(particle.particle_position)

            # Update Velocity und Position
            for particle in self.particles:
                particle.update_velocity(self.global_best_position)
                particle.update_position()

            print(f"[Iteration {iteration+1}/{self.max_iterations}] Global best fitness: {self.global_best_fitness:.4f}")

        return self.global_best_position, self.global_best_fitness
