#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265359
#endif
#define MIN(a,b) (((a)<(b))?(a):(b))

typedef struct 
{
    // [Range, Bearing, Type]
    double (*measurements)[3];
    int n_measurements;
    double *measurement_cov;
} landmark_measurements;

__device__ double mod_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

__device__ double vecnorm(double *v) {
    return sqrt(v[0]*v[0] + v[1]*v[1]);
}

__device__ bool in_sensor_range(double *position, double *landmark, double range, double fov) {
    double x = position[0];
    double y = position[1];
    double theta = position[2];
    double lx = landmark[0];
    double ly = landmark[1];

    double va[] = {lx - x, ly - y};
    double vb[] = {range * cos(theta), range * sin(theta)};

    if(vecnorm(va) > range) {
        return false;
    }

    double angle = acos(
        (va[0]*vb[0] + va[1]*vb[1])/(vecnorm(va)*vecnorm(vb))
    );

    return (angle <= (fov/2));
}

__device__ void to_coords(double *particle, double *in, double *out) {
    double x = particle[0];
    double y = particle[1];
    double theta = particle[2];

    double range = in[0];
    double bearing = in[1];

    out[0] = x + range * cos(bearing + theta);
    out[1] = y + range * sin(bearing + theta);
}

__device__ double* get_particle(double *particles, int i, int max_landmarks) {
    // Stride = 6 (pose) + 8 * max_landmarks
    return (particles + (6 + 8 * max_landmarks) * i);
}

__device__ double* get_mean(double *particle, int i)
{
    return (particle + 6 + 2*i);
}

__device__ double* get_cov(double *particle, int i, int max_landmarks)
{
    return (particle + 6 + 2*max_landmarks + 4*i);
}

__device__ double* get_landmark_prob(double *particle, int i, int max_landmarks)
{
    return (particle + 6 + 6*max_landmarks + i);
}

__device__ double* get_landmark_color(double *particle, int i, int max_landmarks)
{
    return (particle + 6 + 7*max_landmarks + i);
}

// --- TUNED SCORING LOGIC ---
__device__ void increment_landmark_prob(double *particle, int i, int max_landmarks)
{
    double *prob = get_landmark_prob(particle, i, max_landmarks);
    if(prob[0] < 50.0) { // Cap at 50 (solid confidence)
        prob[0] += 4.0;  // Strong Reward: 1 hit buys ~4 misses
    }
}

__device__ void decrement_landmark_prob(double *particle, int i, int max_landmarks)
{
    double *prob = get_landmark_prob(particle, i, max_landmarks);
    prob[0] -= 0.7;      // Gentle Punishment
}
// ---------------------------

__device__ int get_n_landmarks(double *particle)
{
    return (int)particle[5];
}

__device__ void add_landmark(double *particle, double mean[2], double *cov, double type, int max_landmarks)
{
    int n_landmarks = (int)particle[5];
    if (n_landmarks >= max_landmarks) return;

    particle[5] = (double)(n_landmarks + 1);

    double *new_mean = get_mean(particle, n_landmarks);
    double *new_cov = get_cov(particle, n_landmarks, max_landmarks);
    double *new_prob = get_landmark_prob(particle, n_landmarks, max_landmarks);
    double *new_color = get_landmark_color(particle, n_landmarks, max_landmarks);

    new_mean[0] = mean[0];
    new_mean[1] = mean[1];

    new_cov[0] = cov[0];
    new_cov[1] = cov[1];
    new_cov[2] = cov[2];
    new_cov[3] = cov[3];

    new_prob[0] = 15.0;   // Start with 15 lives so it doesn't die instantly
    new_color[0] = type; 
}

__device__ void remove_landmark(double *particle, int i, int max_landmarks)
{
    int n_landmarks = (int)particle[5];
    if (n_landmarks <= 0) return;

    double *mean_a = get_mean(particle, i);
    double *mean_b = get_mean(particle, n_landmarks - 1);

    mean_a[0] = mean_b[0];
    mean_a[1] = mean_b[1];

    double *cov_a = get_cov(particle, i, max_landmarks);
    double *cov_b = get_cov(particle, n_landmarks - 1, max_landmarks);

    cov_a[0] = cov_b[0];
    cov_a[1] = cov_b[1];
    cov_a[2] = cov_b[2];
    cov_a[3] = cov_b[3];

    double *prob_a = get_landmark_prob(particle, i, max_landmarks);
    double *prob_b = get_landmark_prob(particle, n_landmarks - 1, max_landmarks);

    prob_a[0] = prob_b[0];

    // Swap Colors
    double *color_a = get_landmark_color(particle, i, max_landmarks);
    double *color_b = get_landmark_color(particle, n_landmarks - 1, max_landmarks);
    color_a[0] = color_b[0];
    
    particle[5] = (double)(n_landmarks - 1);
}

__device__ void vecmul(double *A, double *u, double *v)
{
    double a = A[0]; double b = A[1]; double c = A[2]; double d = A[3];
    double e = u[0]; double f = u[1];
    v[0] = a*e + b*f;
    v[1] = c*e + d*f;
}

__device__ void matmul(double *A, double *B, double *C)
{
    double a = A[0]; double b = A[1]; double c = A[2]; double d = A[3];
    double e = B[0]; double f = B[1]; double g = B[2]; double h = B[3];
    C[0] = a*e + b*g; C[1] = a*f + b*h;
    C[2] = c*e + d*g; C[3] = c*f + d*h;
}

__device__ void matmul_jacobian(double *H, double *E, double *R, double *S)
{
    double a = H[0]; double b = H[1]; double c = H[2]; double d = H[3];
    double Ht[] = { a, c, b, d };
    matmul(H, E, S);
    matmul(S, Ht, S);
    S[0] += R[0]; S[1] += R[1]; S[2] += R[2]; S[3] += R[3];
}

__device__ void pinv(double *A, double *B)
{
    double a = A[0]; double b = A[1]; double c = A[2]; double d = A[3];
    double det = a*d - b*c;
    double scalar = (fabs(det) > 1e-9) ? 1.0/det : 0.0; 

    B[0] = scalar * d;
    B[1] = scalar * (-b);
    B[2] = scalar * (-c);
    B[3] = scalar * a;
}

__device__ double pdf(double *x, double *mean, double* cov)
{
    double cov_inv[] = {0, 0, 0, 0};
    pinv(cov, cov_inv);

    double det = cov[0]*cov[3] - cov[1]*cov[2];
    double scalar = (det > 0) ? 1.0/(2*M_PI*sqrt(det)) : 0.0;

    double m = x[0] - mean[0];
    double n = mod_angle(x[1] - mean[1]);

    double arg = m*m*(cov_inv[0]) + n*n*(cov_inv[3]) + m*n*(cov_inv[1] + cov_inv[2]);

    return scalar * exp(-0.5 * arg);
}

__device__ void add_measurement_as_landmark(double *particle, double *measurement, double *measurement_cov, int max_landmarks)
{
    double pos[] = { particle[0], particle[1] };
    double landmark[] = {0, 0};
    to_coords(particle, measurement, landmark);

    double q_sq = (landmark[0] - pos[0])*(landmark[0] - pos[0]) + (landmark[1] - pos[1])*(landmark[1] - pos[1]);
    double q = sqrt(q_sq);
    
    if (q < 1e-6) return; 

    double H[] = {
        (landmark[0] - pos[0])/q, (landmark[1] - pos[1])/q,
        -(landmark[1] - pos[1])/q_sq, (landmark[0] - pos[0])/q_sq
    };

    double H_pinv[] = {0,0,0,0};
    pinv(H, H_pinv);

    double H_inv_t[] = { H_pinv[0], H_pinv[2], H_pinv[1], H_pinv[3] };

    double S[] = { 0, 0, 0, 0 };

    matmul(H_pinv, measurement_cov, S);
    matmul(S, H_inv_t, S);

    double type = measurement[2]; 
    add_landmark(particle, landmark, S, type, max_landmarks);
}


__device__ void add_measurements_as_landmarks(double *particle, landmark_measurements *measurements, int max_landmarks)
{
    int n_measurements = measurements->n_measurements;
    double *measurement_cov = measurements->measurement_cov;

    for(int i = 0; i < n_measurements; i++) {
        double R_far[] = { <<FAR_VAR_R>>, 0.0, 0.0, <<FAR_VAR_B>> };
        double *R_i = (measurements->measurements[i][0] > <<FAR_RANGE>>) ? R_far : measurement_cov;
        add_measurement_as_landmark(particle, measurements->measurements[i], R_i, max_landmarks);
    }
}


__device__ double compute_dist(double *particle, int i, double *measurement, double *measurement_cov)
{
    double *landmark = get_mean(particle, i);
    double measurement_xy[] = {0, 0};
    to_coords(particle, measurement, measurement_xy);

    double dist = sqrt(
        (landmark[0] - measurement_xy[0])*(landmark[0] - measurement_xy[0]) +
        (landmark[1] - measurement_xy[1])*(landmark[1] - measurement_xy[1])
    );
    
    return dist;
}


__device__ void update_landmarks(int id, double *particle, landmark_measurements *measurements, int *in_range, int *n_matches, double range, double fov, double thresh, int max_landmarks)
{
    double *measurement_cov = measurements->measurement_cov;
    int n_measurements = measurements->n_measurements;

    int n_landmarks = get_n_landmarks(particle);
    if (n_landmarks > max_landmarks) n_landmarks = max_landmarks;

    int n_in_range = 0;
    for(int i = 0; i < n_landmarks; i++) {
        n_matches[i] = 0;
        double *mean = get_mean(particle, i);
        // --- NEW LOGIC: Calculate Distance to Landmark ---
        double dx = mean[0] - particle[0];
        double dy = mean[1] - particle[1];
        double range_to_lm = sqrt(dx*dx + dy*dy);
        // Filter 1: If the cone is beyond our reliable processing range (25m), 
        // DO NOT CONSIDER IT FOR DELETION. It keeps its score. Before this, it was deleting cones that went more than 25m from car
        if (range_to_lm > 25.0) { 
             continue;
        }
        // Filter 2: Only if it is close enough (<= 25m), check if it's in the FOV.
        // Field of View Check
        if(in_sensor_range(particle, mean, range, fov)) {
            in_range[n_in_range] = i;
            n_in_range++;
        }
    }

    for(int i = 0; i < n_measurements; i++) {
        double R_far[] = { <<FAR_VAR_R>>, 0.0, 0.0, <<FAR_VAR_B>> };
        double *R_i = (measurements->measurements[i][0] > <<FAR_RANGE>>) ? R_far : measurement_cov;
        double best = 1000000.0;
        int best_idx = -1;

        for(int j = 0; j < n_in_range; j++) {
            double lm_type = get_landmark_color(particle, in_range[j], max_landmarks)[0];
            double ms_type = measurements->measurements[i][2];
            if (lm_type != ms_type && lm_type != 0.0 && ms_type != 0.0) continue;
            double dist = compute_dist(particle, in_range[j], measurements->measurements[i], measurement_cov);

            if(dist <= thresh && dist < best) {
                best = dist;
                best_idx = in_range[j];
            }
        }

        if(best_idx != -1) {
            n_matches[best_idx]++;

            double *landmark = get_mean(particle, best_idx);
            double pos[] = { particle[0], particle[1] };
            double theta = particle[2];

            double dx = landmark[0] - pos[0];
            double dy = landmark[1] - pos[1];
            double q_sq = dx*dx + dy*dy;
            double q = sqrt(q_sq);

            double measurement_predicted[] = {
                q, mod_angle(atan2(dy, dx) - theta)
            };

            double residual[2] = {
                measurements->measurements[i][0] - measurement_predicted[0],
                mod_angle(measurements->measurements[i][1] - measurement_predicted[1])
            };

            double H[] = {
                dx/q, dy/q,
                -dy/q_sq, dx/q_sq
            };

            double Ht[] = { H[0], H[2], H[1], H[3] };
            double S[] = { 0, 0, 0, 0 };

            double *landmark_cov = get_cov(particle, best_idx, max_landmarks);
        
            matmul_jacobian(H, landmark_cov, R_i, S);
            double S_inv[] = {0, 0, 0, 0};
            pinv(S, S_inv);

            double Q[] = {0, 0, 0, 0};
            double K[] = { 0, 0, 0, 0 };
            matmul(landmark_cov, Ht, Q);
            matmul(Q, S_inv, K);

            double K_residual[] = { 0, 0 };
            vecmul(K, residual, K_residual);
            landmark[0] += K_residual[0];
            landmark[1] += K_residual[1];

            double KH[] = { 0, 0, 0, 0};
            matmul(K, H, KH);
            double new_cov[] = { 1 - KH[0], -KH[1], -KH[2], 1 - KH[3] };
            matmul(new_cov, landmark_cov, new_cov);
            
            landmark_cov[0] = new_cov[0];
            landmark_cov[1] = new_cov[1];
            landmark_cov[2] = new_cov[2];
            landmark_cov[3] = new_cov[3];
            
            // --- REDUCED INFLATION ---
            // 1e-6 (micro-uncertainty) allows sliding without drifting
            double inflation = 0.000001; 
            landmark_cov[0] += inflation; 
            landmark_cov[3] += inflation;

            particle[3] *= pdf(measurements->measurements[i], measurement_predicted, S);

            increment_landmark_prob(particle, best_idx, max_landmarks);

        } else {
            if (<<NEW_LM_D2>> > 0.0) {
                double det_r = R_i[0] * R_i[3] - R_i[1] * R_i[2];
                particle[3] *= exp(-0.5 * <<NEW_LM_D2>>) / (2.0 * M_PI * sqrt(det_r));
            }
            add_measurement_as_landmark(particle, measurements->measurements[i], R_i, max_landmarks);
        }
    }

    // Cleanup loop for Negative Information
    for(int i = n_in_range - 1; i >= 0; i--) {
        int idx = in_range[i];
        
        if(n_matches[idx] == 0) {
            // --- NEW SAFE ZONE LOGIC ---
            // Calculate distance again
            double *lm = get_mean(particle, idx);
            double dist_sq = (lm[0]-particle[0])*(lm[0]-particle[0]) + (lm[1]-particle[1])*(lm[1]-particle[1]);
            // If the cone is further than 10 meters, DO NOT punish it.
            // Just skip the decrement. It stays alive safe and sound.
            if (dist_sq > (10.0 * 10.0)) {
                continue;
            }

            decrement_landmark_prob(particle, idx, max_landmarks);
            double *prob_ptr = get_landmark_prob(particle, idx, max_landmarks);
            
            if(prob_ptr[0] < 0.0) {
                remove_landmark(particle, idx, max_landmarks);
            }
        } 
    }
}

__global__ void update(
    double *particles, int block_size, int *scratchpad_mem, int scratchpad_size, double measurements_array[][3], int n_particles, int n_measurements,
    double *measurement_cov, double threshold, double range, double fov, int max_landmarks)
{
    if(n_measurements == 0) {
        return;
    }

    int block_id = blockIdx.x + blockIdx.y * gridDim.x;
    int thread_id = block_id * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

    int *scratchpad = scratchpad_mem + (2 * thread_id * max_landmarks);
    int *in_range = scratchpad;
    int *n_matches = in_range + max_landmarks;

    landmark_measurements measurements;
    measurements.n_measurements = n_measurements;
    measurements.measurement_cov = measurement_cov;
    measurements.measurements = measurements_array;

    for(int k = 0; k < block_size; k++) {
        int particle_id = thread_id*block_size + k;
        if(particle_id >= n_particles) {
            return;
        }
        
        double *particle = get_particle(particles, particle_id, max_landmarks);
        int n_landmarks = get_n_landmarks(particle);
    
        if(n_landmarks == 0) {
            add_measurements_as_landmarks(particle, &measurements, max_landmarks);
            continue;
        }

        update_landmarks(particle_id, particle, &measurements, in_range, n_matches, range, fov, threshold, max_landmarks);
    }
}