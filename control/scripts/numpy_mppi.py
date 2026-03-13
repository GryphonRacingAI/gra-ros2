import numpy as np
import matplotlib.pyplot as plt

# ============================
# Parameters
# ============================
dt = 0.05
H = 16          # Horizon
K = 200         # Number of rollouts
lambda_ = 2.0
max_iters = 1500
lookahead_dist = 0.50

# Control noise (base & minimum for adaptive sigma)
sigma_u_base = np.array([0.6, 0.15])
sigma_u_min  = np.array([0.2, 0.05])

# Weights
w_path, w_heading = 20, 3
w_speed, w_control = 10, 1.5
w_terminal, w_obstacle = 5, 15

# Vehicle limits
MAX_A   = 3.0      # [m/s^2]
MIN_V   = -1.0     # [m/s]
MAX_V   = 8.0      # [m/s]
MAX_STEER = np.pi/6  # [rad] ~0.52

# Initial state: x, y, heading, velocity
x = np.array([7, 44, 0, 0.0], dtype=float)

# ============================
# Load cones & build path
# ============================
inner_cones = np.loadtxt('inner_cones.csv', delimiter=',')
outer_cones = np.loadtxt('outer_cones.csv', delimiter=',')
n = min(len(inner_cones), len(outer_cones))
inner_cones = inner_cones[:n]
outer_cones = outer_cones[:n]

# # Optional: densify cones if needed
def densify(cones, min_extra=3, max_extra=4):
    new_pts = []
    for i in range(len(cones)-1):
        p0 = cones[i]
        p1 = cones[i+1]
        new_pts.append(p0)
        m = np.random.randint(min_extra, max_extra+1)
        if m > 0:
            ts = np.linspace(0, 1, m+2)[1:-1]
            inter = p0 + ts[:, None] * (p1 - p0)
            new_pts.extend(inter)
    new_pts.append(cones[-1])
    return np.vstack(new_pts)

inner_cones = densify(inner_cones, 3, 3)
outer_cones = densify(outer_cones, 3, 3)

# Centerline path
path = (inner_cones + outer_cones) / 2.0
N_path = path.shape[0]

# ============================
# Curvature-based reference speed profile
# ============================
dx_path = np.gradient(path[:, 0])
dy_path = np.gradient(path[:, 1])
ddx_path = np.gradient(dx_path)
ddy_path = np.gradient(dy_path)

# curvature kappa
kappa = (dx_path * ddy_path - dy_path * ddx_path) / (dx_path**2 + dy_path**2 + 1e-6)**1.5
kappa_abs = np.abs(kappa)

a_lat_max = 2.0         # max comfortable lateral accel [m/s^2]
v_max_straight = 7.0    # top speed on straight
v_min = 0.2             # avoid zero

v_ref_path = np.sqrt(a_lat_max / (kappa_abs + 1e-3))  # avoid inf on straights
v_ref_path = np.clip(v_ref_path, v_min, v_max_straight)

# Precompute path heading (for heading cost)
heading_path = np.arctan2(dy_path, dx_path)

# ============================
# Obstacles (cones) with radii
# ============================
r1 = 0.2 * np.ones(len(inner_cones))
r2 = 0.2 * np.ones(len(outer_cones))
obs = np.vstack([
    np.hstack([inner_cones, r1[:, None]]),
    np.hstack([outer_cones, r2[:, None]])
])

# ============================
# Helper functions
# ============================
def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

def dynamics_vec(X, U):
    """
    Vectorized dynamics:
    X: (K, H+1, 4)  [x, y, theta, v]
    U: (K, H, 2)    [accel, steer_rate-like]
    """
    K, H, _ = U.shape
    for h in range(H):
        a = clamp(U[:, h, 0], -MAX_A, MAX_A)
        steer = clamp(U[:, h, 1], -MAX_STEER, MAX_STEER)
        theta = X[:, h, 2]
        v = X[:, h, 3]

        # simple kinematic model
        X[:, h+1, 0] = X[:, h, 0] + dt * v * np.cos(theta)
        X[:, h+1, 1] = X[:, h, 1] + dt * v * np.sin(theta)
        X[:, h+1, 2] = theta + dt * steer
        X[:, h+1, 3] = clamp(v + dt * a, MIN_V, MAX_V)

    return X

# ============================
# MPPI setup
# ============================
u0 = np.zeros((H, 2))  # nominal controls
# small bias to move forward at the start
u0[:, 0] = 1.0
u0[:, 1] = 0.0

history_state = np.full((4, max_iters), np.nan)
history_u = np.full((max_iters, 2), np.nan)
history_cost = np.full(max_iters, np.nan)

# Start at first path index
idx = 0
search_window = 50  # how far ahead we search for closest point

# ============================
# MPPI loop
# ============================
for t in range(max_iters):
    # ------------------------
    # 1. Find closest path point (monotonic forward)
    # ------------------------
    i0 = idx
    i1 = min(idx + search_window, N_path)
    dists = np.linalg.norm(path[i0:i1] - x[:2], axis=1)
    idx_local = np.argmin(dists)
    idx = i0 + idx_local

    # ------------------------
    # 2. Reference path & speed horizon
    # ------------------------
    # path points
    if idx + H + 1 <= N_path:
        ref_pts = path[idx:idx+H+1]
    else:
        # pad with last point
        ref_pts = np.vstack([
            path[idx:],
            np.tile(path[-1], (idx + H + 1 - N_path, 1))
        ])

    # heading reference
    if idx + H <= N_path:
        heading_ref = heading_path[idx:idx+H]
    else:
        extra = idx + H - N_path
        heading_ref = np.hstack([
            heading_path[idx:],
            np.full(extra, heading_path[-1])
        ])

    # speed reference
    if idx + H + 1 <= N_path:
        v_ref = v_ref_path[idx:idx+H+1]
    else:
        v_ref = np.hstack([
            v_ref_path[idx:],
            np.full(idx + H + 1 - N_path, v_ref_path[-1])
        ])

    # ------------------------
    # 3. Sample noise (adaptive sigma + time correlation)
    # ------------------------
    frac = t / max(1, (max_iters - 1))
    sigma_u_t = sigma_u_base * (1 - frac) + sigma_u_min * frac

    noise = np.random.randn(K, H, 2) * sigma_u_t

    # time-correlated noise
    alpha = 0.5
    for h in range(1, H):
        noise[:, h, :] = alpha * noise[:, h-1, :] + (1 - alpha) * noise[:, h, :]

    U_rollouts = u0[None, :, :] + noise

    # ------------------------
    # 4. Simulate dynamics
    # ------------------------
    X = np.zeros((K, H+1, 4))
    X[:, 0, :] = x
    X = dynamics_vec(X, U_rollouts)

    # ------------------------
    # 5. Cost computation
    # ------------------------

    # Path deviation
    d_path = np.linalg.norm(X[:, :, :2] - ref_pts[None, :, :], axis=2)
    # Skip step 0 since it's the current state
    path_cost = w_path * np.sum(d_path[:, 1:], axis=1)

    # Heading error
    heading_err = normalize_angle(X[:, 1:, 2] - heading_ref[None, :])
    heading_cost = w_heading * np.sum(np.abs(heading_err), axis=1)

    cost = path_cost + heading_cost

    # Control effort
    control_cost = w_control * np.sum(U_rollouts**2, axis=(1, 2))
    cost += control_cost

    # Terminal cost
    terminal_cost = w_terminal * np.linalg.norm(X[:, -1, :2] - ref_pts[-1], axis=1)
    cost += terminal_cost

    # Obstacle cost
    X_pos = X[:, :, :2][:, :, None, :]       # (K, H+1, 1, 2)
    obs_pos = obs[:, :2][None, None, :, :]   # (1, 1, num_obs, 2)
    obs_r = obs[:, 2][None, None, :]         # (1, 1, num_obs)

    d_obs = np.linalg.norm(X_pos - obs_pos, axis=-1) - obs_r  # (K, H+1, num_obs)

    safety_dist = 0.6
    d_pen = safety_dist - d_obs           # > 0 when too close
    d_pen = np.clip(d_pen, 0.0, None)
    obs_cost = w_obstacle * np.sum(d_pen**2, axis=(1, 2))
    # cap extreme obstacle cost to avoid numerical issues
    obs_cost = np.clip(obs_cost, 0.0, 1e4)
    cost += obs_cost

    # Speed cost (asymmetric: punish only overspeed)
    v_traj = X[:, :, 3]                   # (K, H+1)
    speed_err = v_traj - v_ref[None, :]   # positive => too fast
    over_speed = np.clip(speed_err, 0.0, None)
    speed_cost = w_speed * np.sum(over_speed[:, 1:]**2, axis=1)
    cost += speed_cost

    # Optional global soft maximum speed (uncomment if you want it)
    # v_soft_max = 7.0
    # v_excess_global = np.clip(v_traj - v_soft_max, 0.0, None)
    # cost += 2.0 * np.sum(v_excess_global**2, axis=1)

    # ------------------------
    # 6. MPPI weight update
    # ------------------------
    S_min = cost.min()
    weights = np.exp(-(cost - S_min) / lambda_)
    weights_sum = weights.sum() + 1e-12
    weights /= weights_sum

    du = np.sum(weights[:, None, None] * noise, axis=0)
    u0 += du
    u0[:, 0] = clamp(u0[:, 0], -MAX_A, MAX_A)
    u0[:, 1] = clamp(u0[:, 1], -MAX_STEER, MAX_STEER)

    # ------------------------
    # 7. Apply first control to the real state
    # ------------------------
    a = clamp(u0[0, 0], -MAX_A, MAX_A)
    steer = clamp(u0[0, 1], -MAX_STEER, MAX_STEER)

    theta = x[2]
    v = x[3]

    x[0] += dt * v * np.cos(theta)
    x[1] += dt * v * np.sin(theta)
    x[2] = normalize_angle(theta + dt * steer)
    x[3] = clamp(v + dt * a, MIN_V, MAX_V)

    # Shift control sequence (receding horizon)
    u0 = np.vstack([u0[1:], np.zeros((1, 2))])

    # ------------------------
    # 8. Log history
    # ------------------------
    history_state[:, t] = x
    history_u[t, :] = np.array([a, steer])
    history_cost[t] = cost.mean()

    if t % 10 == 0:
        print(f'Step {t+1}: pos=({x[0]:.2f}, {x[1]:.2f}), speed={x[3]:.2f}, idx={idx}')

    # ------------------------
    # 9. Check path completion
    # ------------------------
    # For a closed track you might remove this and just run forever.
    if idx >= N_path - 2 and np.linalg.norm(x[:2] - path[-1]) < lookahead_dist:
        print(f'Path complete at step {t+1}')
        history_state = history_state[:, :t+1]
        history_u = history_u[:t+1, :]
        history_cost = history_cost[:t+1]
        break

print('Simulation complete.')

# ============================
# PLOTS
# ============================
# Trajectory
plt.figure(figsize=(10, 6))
plt.plot(path[:, 0], path[:, 1], 'k--', label='Reference Path', linewidth=2)
plt.scatter(inner_cones[:, 0], inner_cones[:, 1], color='red', label='Inner cones', s=5)
plt.scatter(outer_cones[:, 0], outer_cones[:, 1], color='green', label='Outer cones', s=5)
plt.plot(history_state[0, :], history_state[1, :], 'b-', linewidth=2, label='Vehicle trajectory')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('MPPI Trajectory')
plt.axis('equal')
plt.legend()
plt.grid(True)

# Speed profile
plt.figure(figsize=(10, 4))
plt.plot(history_state[3, :], 'b-', linewidth=2)
plt.xlabel('Time step')
plt.ylabel('Speed [m/s]')
plt.title('Vehicle Speed over Time')
plt.grid(True)

# Control inputs
plt.figure(figsize=(10, 4))
plt.plot(history_u[:, 0], 'r-', label='Acceleration')
plt.plot(history_u[:, 1], 'g-', label='Steering rate')
plt.xlabel('Time step')
plt.ylabel('Control input')
plt.title('Control Inputs over Time')
plt.legend()
plt.grid(True)

# Cost evolution
plt.figure(figsize=(10, 4))
plt.plot(history_cost, 'm-', linewidth=2)
plt.xlabel('Time step')
plt.ylabel('Average rollout cost')
plt.title('Cost Evolution')
plt.grid(True)

plt.show()
