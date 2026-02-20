import numpy as np
import matplotlib.pyplot as plt

# ============================
# Parameters
# ============================
dt = 0.05
H = 16         # Horizon
K = 200        # Number of rollouts
lambda_ = 2.0
max_iters = 800
sigma_u = np.array([0.6, 0.15])
lookahead_dist = 0.50

# Weights
w_path, w_heading, w_control, w_terminal, w_obstacle = 200, 3, 2, 5, 200

# Initial state: x, y, heading, velocity
x = np.array([7.5, 44, 0, 0], dtype=float)

# Load cones
inner_cones = np.loadtxt('inner_cones.csv', delimiter=',')
outer_cones = np.loadtxt('outer_cones.csv', delimiter=',')
n = min(len(inner_cones), len(outer_cones))
inner_cones = inner_cones[:n]
outer_cones = outer_cones[:n]

def densify(cones, min_extra=3, max_extra=4):
    new_pts = []
    for i in range(len(cones)-1):
        p0 = cones[i]
        p1 = cones[i+1]
        new_pts.append(p0)
        m = np.random.randint(min_extra, max_extra+1)
        if m > 0:
            ts = np.linspace(0, 1, m+2)[1:-1]          # interpolation fractions
            inter = p0 + ts[:, None] * (p1 - p0)      # (m,2)
            new_pts.extend(inter)
    new_pts.append(cones[-1])
    return np.vstack(new_pts)

inner_cones = densify(inner_cones, 4, 4)
outer_cones = densify(outer_cones, 4, 4)
path = (inner_cones + outer_cones) / 2

# Obstacles (cones) with radii
r1, r2 = 0.2*np.ones(len(inner_cones)), 0.2*np.ones(len(outer_cones))
obs = np.vstack([np.hstack([inner_cones, r1[:,None]]), 
                 np.hstack([outer_cones, r2[:,None]])])

# Nominal controls
u0 = np.zeros((H, 2))

# History logs
history_state = np.full((4, max_iters), np.nan)
history_u = np.full((max_iters, 2), np.nan)
history_cost = np.full(max_iters, np.nan)

# ============================
# Helper functions
# ============================
def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

def dynamics_vec(X, U):
    """Vectorized dynamics: X shape (K, H+1, 4), U shape (K, H, 2)"""
    K, H, _ = U.shape
    for h in range(H):
        a = clamp(U[:,h,0], -3, 3)
        steer = clamp(U[:,h,1], -0.6, 0.6)
        theta = X[:,h,2]
        v = X[:,h,3]
        X[:,h+1,0] = X[:,h,0] + dt * v * np.cos(theta)
        X[:,h+1,1] = X[:,h,1] + dt * v * np.sin(theta)
        X[:,h+1,2] = theta + dt * steer
        X[:,h+1,3] = clamp(v + dt * a, -1, 6)
    return X

# ============================
# MPPI loop
# ============================
for t in range(max_iters):
    # 1. Find closest path point
    dists = np.linalg.norm(path - x[:2], axis=1)
    idx = np.argmin(dists)
    
    # 2. Sample noise
    noise = np.random.randn(K, H, 2) * sigma_u
    U_rollouts = u0[None,:,:] + noise
    
    # 3. Initialize trajectories
    X = np.zeros((K, H+1, 4))
    X[:,0,:] = x
    
    # 4. Simulate dynamics
    X = dynamics_vec(X, U_rollouts)
    
    # 5. Compute reference points for horizon
    ref_pts = path[idx:idx+H+1] if idx+H+1 <= path.shape[0] else path[idx:]
    if len(ref_pts) < H+1:
        ref_pts = np.vstack([ref_pts, np.tile(path[-1], (H+1-len(ref_pts),1))])
    
    # -------------------
    # Path and heading cost
    # -------------------
    d_path = np.linalg.norm(X[:,:,:2] - ref_pts[None,:,:], axis=2)
    heading_ref = np.arctan2(ref_pts[1:,1]-ref_pts[:-1,1], ref_pts[1:,0]-ref_pts[:-1,0])
    heading_err = np.abs(normalize_angle(X[:,1:,2] - heading_ref[None,:]))
    cost = w_path*np.sum(d_path[:,1:], axis=1) + w_heading*np.sum(heading_err, axis=1)
    
    # -------------------
    # Control cost
    # -------------------
    cost += w_control*np.sum(U_rollouts**2, axis=(1,2))
    
    # -------------------
    # Terminal cost
    # -------------------
    cost += w_terminal*np.linalg.norm(X[:,-1,:2] - ref_pts[-1], axis=1)
    
    # -------------------
    # Obstacle cost (vectorized)
    # -------------------
    X_pos = X[:,:,:2][:,:,None,:]       # (K, H+1, 1, 2)
    obs_pos = obs[:, :2][None, None, :, :]  # (1, 1, num_obs, 2)
    obs_r = obs[:, 2][None, None, :]        # (1, 1, num_obs)
    d_obs = np.linalg.norm(X_pos - obs_pos, axis=-1) - obs_r
    # Penalize approaching obstacles within a safety distance (soft penalty),
    # not just collisions. Tune safety_dist as needed.
    safety_dist = 0.6
    d_pen = safety_dist - d_obs          # positive when closer than safety_dist
    d_obs = np.clip(d_pen, 0.0, None)
    cost += w_obstacle * np.sum(d_obs**2, axis=(1,2))
    
    # -------------------
    # MPPI weights
    # -------------------
    S_min = cost.min()
    weights = np.exp(-(cost - S_min)/lambda_)
    weights /= weights.sum() + 1e-12
    
    du = np.sum(weights[:,None,None] * noise, axis=0)
    u0 += du
    u0[:,0] = clamp(u0[:,0], -3, 3)
    u0[:,1] = clamp(u0[:,1], -0.6, 0.6)
    
    # 6. Apply first control
    a = clamp(u0[0,0], -3, 3)
    steer = clamp(u0[0,1], -np.pi/4, np.pi/4)
    theta = x[2]
    v = x[3]
    x[0] += dt * v * np.cos(theta)
    x[1] += dt * v * np.sin(theta)
    x[2] += dt * steer
    x[3] = clamp(v + dt * a, -1, 5) # clamp speed
    u0 = np.vstack([u0[1:], np.zeros((1,2))])
    
    # 7. Log history
    history_state[:,t] = x
    history_u[t] = u0[0]
    history_cost[t] = cost.mean()
    
    print(f'Step {t+1}: pos=({x[0]:.2f}, {x[1]:.2f}), speed={x[3]:.2f}')
    
    # Move along path
    if np.linalg.norm(x[:2]-path[idx]) < lookahead_dist:
        idx = min(idx+1, path.shape[0]-1)
    if idx >= path.shape[0]-1:
        print(f'Path complete at step {t+1}')
        history_state = history_state[:,:t+1]
        history_u = history_u[:t+1]
        history_cost = history_cost[:t+1]
        break

print('Simulation complete.')

# ============================
# PLOTS
# ============================
# Trajectory
plt.figure(figsize=(10,6))
plt.plot(path[:,0], path[:,1], 'k--', label='Reference Path', linewidth=2)
plt.scatter(inner_cones[:,0], inner_cones[:,1], color='red', label='Inner cones')
plt.scatter(outer_cones[:,0], outer_cones[:,1], color='green', label='Outer cones')
plt.plot(history_state[0,:], history_state[1,:], 'b-', linewidth=2, label='Vehicle trajectory')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('MPPI Trajectory')
plt.axis('equal')
plt.legend()
plt.grid(True)

# Speed profile
plt.figure(figsize=(10,4))
plt.plot(history_state[3,:], 'b-', linewidth=2)
plt.xlabel('Time step')
plt.ylabel('Speed [m/s]')
plt.title('Vehicle Speed over Time')
plt.grid(True)

# Control inputs
plt.figure(figsize=(10,4))
plt.plot(history_u[:,0], 'r-', label='Acceleration')
plt.plot(history_u[:,1], 'g-', label='Steering')
plt.xlabel('Time step')
plt.ylabel('Control input')
plt.title('Control Inputs over Time')
plt.legend()
plt.grid(True)

# Cost evolution
plt.figure(figsize=(10,4))
plt.plot(history_cost, 'm-', linewidth=2)
plt.xlabel('Time step')
plt.ylabel('Average rollout cost')
plt.title('Cost Evolution')
plt.grid(True)

plt.show()
