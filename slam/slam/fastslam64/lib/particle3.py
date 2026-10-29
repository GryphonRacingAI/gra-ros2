import math
import numpy as np
from numpy.random import random
import time

def systematic_resample(weights):
    N = weights.shape[0]

    # make N subdivisions, and choose positions with a consistent random offset
    positions = (random() + np.arange(N)) / N

    indexes = np.zeros(N, dtype=np.int32)
    cumulative_sum = np.cumsum(weights)
    # prevent float imprecision
    cumulative_sum[-1] = 1.0

    i, j = 0, 0
    while i < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes


class FlatParticle(object):
    # CHANGED: All steps are now 6 + 8 * max_landmarks (Header + 2Mean + 4Cov + 1Prob + 1Color)
    
    @staticmethod
    def x(particles):
        max_landmarks = int(particles[4])
        step = 6 + 8*max_landmarks 
        return particles[0::step]

    @staticmethod
    def y(particles):
        max_landmarks = int(particles[4])
        step = 6 + 8*max_landmarks
        return particles[1::step]

    @staticmethod
    def w(particles):
        max_landmarks = int(particles[4])
        step = 6 + 8*max_landmarks
        return particles[3::step]

    @staticmethod
    def len(particles):
        max_landmarks = int(particles[4])
        length = particles.shape[0]
        # Safety check to avoid division by zero if initialized wrong
        if length == 0: return 0
        return int(length/(6 + 8*max_landmarks))

    @staticmethod
    def get_particle(particles, i):
        max_landmarks = int(particles[4])
        size = 6 + 8*max_landmarks
        offset = size * i
        return particles[offset:offset+size]

    @staticmethod
    def get_landmarks(particles, i):
        particle = FlatParticle.get_particle(particles, i)
        n_landmarks = int(particle[5])

        return particle[6:6+2*n_landmarks].reshape((n_landmarks, 2))

    @staticmethod
    def get_covariances(particles, i):
        particle = FlatParticle.get_particle(particles, i)
        max_landmarks = int(particle[4])
        n_landmarks = int(particle[5])

        # Covariance starts after Mean (2*ML)
        start = 6 + 2*max_landmarks
        end   = 6 + 6*max_landmarks
        cov_array = particle[start:end]
        
        covariances = np.zeros((n_landmarks, 2, 2), dtype=np.float64)

        for k in range(n_landmarks):
            covariances[k, 0, 0] = cov_array[4*k]
            covariances[k, 0, 1] = cov_array[4*k + 1]
            covariances[k, 1, 0] = cov_array[4*k + 2]
            covariances[k, 1, 1] = cov_array[4*k + 3]

        return covariances

    # --- NEW: Helper to extract colors ---
    @staticmethod
    def get_colors(particles, i):
        particle = FlatParticle.get_particle(particles, i)
        max_landmarks = int(particle[4])
        n_landmarks = int(particle[5])
        
        # Color starts after Probabilities (which is at 6 + 6*ML)
        # So Color starts at 6 + 7*ML
        start = 6 + 7*max_landmarks
        end   = start + n_landmarks # We only want initialized colors
        
        return particle[start:end]

    @staticmethod
    def get_initial_particles(n_particles: int, max_landmarks: int, starting_position: np.ndarray, sigma: float):
        step = 6 + 8*max_landmarks
        particles = np.zeros(n_particles * step, dtype=np.float64)

        particles[0::step] = starting_position[0]
        particles[1::step] = starting_position[1]
        particles[2::step] = starting_position[2]
        particles[3::step] = 1.0/n_particles
        particles[4::step] = float(max_landmarks)

        return particles

    @staticmethod
    def neff(weights) -> float:
        return 1.0/np.sum(np.square(weights))