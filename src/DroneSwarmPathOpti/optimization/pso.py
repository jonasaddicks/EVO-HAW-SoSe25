from copy import deepcopy
from typing import Callable

from DroneSwarmPathOpti.simulation import Environment

from .particle import Particle, DronePath
from ..config import get_settings

settings = get_settings()

class PSO:
    """
    This class contains the logical component of the particle swarm optimization and controls the evolutionary process.
    """

    fitness_function: Callable[[list[DronePath], Environment], float]
    environment: Environment
    num_particles: int
    max_iterations: int

    particles: list[Particle]

    global_best_position: list[DronePath]
    global_best_fitness: float

    def __init__(self, fitness_function, environment: Environment):
        self.fitness_function = fitness_function
        self.environment = environment
        self.num_particles = settings.PSO_PARTICLES
        self.max_iterations = settings.PSO_ITERATIONS

        self.step_decrease_max_velocity_x = (settings.PSO_MAX_VELOCITY_X - settings.PSO_DECREASE_MAX_VELOCITY_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_DECREASE_MAX_VELOCITY_WHEN))
        self.step_decrease_max_velocity_y = (settings.PSO_MAX_VELOCITY_Y - settings.PSO_DECREASE_MAX_VELOCITY_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_DECREASE_MAX_VELOCITY_WHEN))
        self.step_decrease_initial_velocity_x = (settings.PSO_MAX_INITIAL_VELOCITY_X - settings.PSO_DECREASE_INITIAL_VELOCITY_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_DECREASE_INITIAL_VELOCITY_WHEN))
        self.step_decrease_initial_velocity_y = (settings.PSO_MAX_INITIAL_VELOCITY_Y - settings.PSO_DECREASE_INITIAL_VELOCITY_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_DECREASE_INITIAL_VELOCITY_WHEN))

        self.step_increase_weight_global = (settings.PSO_WEIGHT_GLOBAL_BEST - settings.PSO_INCREASE_WEIGHT_GLOBAL_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_INCREASE_WEIGHT_GLOBAL_WHEN))
        self.step_decrease_weight_personal = (settings.PSO_WEIGHT_PERSONAL_POSITION - settings.PSO_DECREASE_WEIGHT_PERSONAL_GOAL) / (self.max_iterations - (self.max_iterations * settings.PSO_DECREASE_WEIGHT_PERSONAL_WHEN))

        self.particles = [Particle() for _ in range(self.num_particles)]

        self.global_best_position = deepcopy(self.particles[0].particle_position)
        self.global_best_fitness = float("inf")

    def optimize(self):
        """
        This method regulates the process of evolution and implements the logic of the particle swarm optimization.

        The amount of iterations is specified by the config.

        :return: A tuple containing the best solution found after the optimization process has been completed and its corresponding fitness value.
        """
        for iteration in range(self.max_iterations):

            # ADJUST PARAMETERS WHILE PROGRESSING
            if iteration > self.max_iterations * settings.PSO_DECREASE_MAX_VELOCITY_WHEN:
                settings.PSO_MAX_VELOCITY_X -= self.step_decrease_max_velocity_x
                settings.PSO_MAX_VELOCITY_Y -= self.step_decrease_max_velocity_y

            if iteration > self.max_iterations * settings.PSO_DECREASE_INITIAL_VELOCITY_WHEN:
                settings.PSO_MAX_INITIAL_VELOCITY_X -= self.step_decrease_initial_velocity_x
                settings.PSO_MAX_INITIAL_VELOCITY_Y -= self.step_decrease_initial_velocity_y

            if iteration > self.max_iterations * settings.PSO_INCREASE_WEIGHT_GLOBAL_WHEN:
                settings.PSO_WEIGHT_GLOBAL_BEST -= self.step_increase_weight_global

            if iteration > self.max_iterations * settings.PSO_DECREASE_WEIGHT_PERSONAL_WHEN:
                settings.PSO_WEIGHT_PERSONAL_POSITION -= self.step_decrease_weight_personal

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

            self.particles.sort(key=lambda p: p.current_fitness)
            if iteration > self.max_iterations * settings.PSO_FLUSH_WHEN:
                for i in range((self.num_particles - 1), int(self.num_particles - self.num_particles * settings.PSO_FLUSH_SHARE), -1):
                    self.particles[i].particle_position = deepcopy(self.global_best_position)
                    self.particles[i].best_position = deepcopy(self.global_best_position)
                    self.particles[i].reset_velocity()

            # Update Velocity und Position
            for particle in self.particles:
                particle.update_velocity(self.global_best_position)
                particle.update_position()

            #print(f"[Iteration {iteration+1}/{self.max_iterations}] Global best fitness: {self.global_best_fitness:.4f}")
            print(f"{self.global_best_fitness:.4f}")

        return self.global_best_position, self.global_best_fitness
