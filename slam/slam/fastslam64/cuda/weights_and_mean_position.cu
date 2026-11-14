#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define N_PARTICLES <<N_PARTICLES>>
#define PARTICLE_SIZE <<PARTICLE_SIZE>>
#define THREADS <<THREADS>>

__device__ double* get_particle(double *particles, int i) {
    return (particles + PARTICLE_SIZE*i);
}

/*
 * Extracts weights from particles.
 */
 __global__ void get_weights(double *particles, double *weights) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    double *particle = get_particle(particles, i);
    weights[i] = (double)particle[3];
}


/*
 * Calculates the mean position of all particles (weighted mean x, y; weighted circular mean heading).
 * Needs to run in a single block.
 */
 __global__ void get_mean_position(double *particles, double *mean) {
    int block_id = blockIdx.x + blockIdx.y * gridDim.x;
    int idx = block_id * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

    double x = 0;
    double y = 0;
    double sin_theta = 0;
    double cos_theta = 0;

    for (int i = idx; i < N_PARTICLES; i += THREADS) {
        double *particle = get_particle(particles, i);
        x += particle[3] * particle[0];
        y += particle[3] * particle[1];
        sin_theta += particle[3] * sin(particle[2]);
        cos_theta += particle[3] * cos(particle[2]);
    }

    __shared__ double r_x[THREADS];
    __shared__ double r_y[THREADS];
    __shared__ double r_sin[THREADS];
    __shared__ double r_cos[THREADS];

    r_x[idx] = x;
    r_y[idx] = y;
    r_sin[idx] = sin_theta;
    r_cos[idx] = cos_theta;

    __syncthreads();

    for (int size = THREADS/2; size>0; size/=2) {
        if (idx<size) {
            r_x[idx] += r_x[idx+size];
            r_y[idx] += r_y[idx+size];
            r_sin[idx] += r_sin[idx+size];
            r_cos[idx] += r_cos[idx+size];
        }
        __syncthreads();
    }

    if (idx == 0) {
        mean[0] = r_x[0];
        mean[1] = r_y[0];
        mean[2] = atan2(r_sin[0], r_cos[0]);
    }
}