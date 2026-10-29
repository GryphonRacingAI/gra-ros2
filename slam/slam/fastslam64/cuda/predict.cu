#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <curand_kernel.h>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define PARTICLE_SIZE <<PARTICLE_SIZE>>
#define N_PARTICLES <<N_PARTICLES>>  // Make sure this is defined for safety checks

__device__ double* get_particle(double *particles, int i) {
    return (particles + PARTICLE_SIZE*i);
}

extern "C" {

// Each thread has a random state
__device__ curandState_t* states[N_PARTICLES];


// This function is only called once to initialize the rngs.
__global__ void init_rng(int seed)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= N_PARTICLES) return; // Safety Check

    curandState_t* s = new curandState_t;
    curand_init(seed, i, 0, s);
    states[i] = s;
}


__global__ void predict_from_imu(double *particles,
    double x, double y, double theta, double sigma_x, double sigma_y, double sigma_theta) {
    
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= N_PARTICLES) return; // Safety Check

    double *particle = get_particle(particles, i);
    
    // curand_normal() samples from standard normal
    // Y = mu + sigma*X
    particle[0] = x + sigma_x * curand_normal(states[i]);
    particle[1] = y + sigma_y * curand_normal(states[i]);
    
    // Update theta with noise and normalize
    particle[2] = theta + sigma_theta * curand_normal(states[i]);
    particle[2] = atan2(sin(particle[2]), cos(particle[2]));
}

// Moves particles based on the control input and movement model.
__global__ void predict_from_model(double *particles, double ua, double ub, double sigma_a, double sigma_b, double dt) {
    if(ua == 0.0 && ub == 0.0) {
        return;
    }

    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= N_PARTICLES) return; // Safety Check

    double *particle = get_particle(particles, i);

    double noisy_ua = ua + sigma_a * curand_normal(states[i]);
    double noisy_ub = ub + sigma_b * curand_normal(states[i]);

    double angle = particle[2];

    // Update Orientation
    particle[2] += (noisy_ua * dt);
    particle[2] = atan2(sin(particle[2]), cos(particle[2]));

    // Update Position
    double dist = (noisy_ub * dt);
    particle[0] += cos(angle) * dist;
    particle[1] += sin(angle) * dist;
}

// Moves particles based on the Formula Student Online model (3-DOF input?)
__global__ void predict_from_fsonline_model(double *particles, double ua, double ub, double uc, double sigma_a, double sigma_b, double dt) {
    if(ua == 0.0 && ub == 0.0 && uc == 0.0) {
        return;
    }

    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i >= N_PARTICLES) return; // Safety Check

    double *particle = get_particle(particles, i);

    double noisy_ua = ua + sigma_a * curand_normal(states[i]);
    double noisy_ub = ub + sigma_b * curand_normal(states[i]);
    double noisy_uc = uc + sigma_a * curand_normal(states[i]);

    // 1. First rotation
    particle[2] += (noisy_ua * dt);
    particle[2] = atan2(sin(particle[2]), cos(particle[2]));

    // 2. Linear motion
    double dist = (noisy_ub * dt);
    particle[0] += cos(particle[2]) * dist;
    particle[1] += sin(particle[2]) * dist;

    // 3. Second rotation (if your model requires it)
    particle[2] += (noisy_uc * dt);
    particle[2] = atan2(sin(particle[2]), cos(particle[2]));
}

}