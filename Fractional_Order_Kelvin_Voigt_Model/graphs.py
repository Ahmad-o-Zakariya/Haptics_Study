import numpy as np
import matplotlib.pyplot as plt

# --- SIMULATION PARAMETERS ---
Ts = 0.001          # 1ms Sampling period
t = np.arange(0, 0.8, Ts)
N = len(t)
x_wall = 400        # Wall Horizon (counts)
K = 8.0             # Stiffness baseline

# Define a realistic hand-penetration profile: entering wall and returning smoothly
# Penetration starts around t = 0.1s, peaks at 460 counts, exits around t = 0.5s
x = 340 + 120 * np.sin(2.5 * t) 
e = np.maximum(0, x - x_wall)

# Parameter configuration matrix
configs = {
    0.5: {'B': 0.70, 'color': '#1f77b4', 'style': '-'},
    1.0: {'B': 0.20, 'color': '#ff7f0e', 'style': '--'},
    1.5: {'B': 0.01, 'color': '#2ca02c', 'style': '-.'}
}

def compute_gl_derivative(error_array, mu_val, sample_time):
    num_pts = len(error_array)
    deriv = np.zeros(num_pts)
    weights = np.zeros(num_pts)
    weights[0] = 1.0
    for j in range(1, num_pts):
        weights[j] = (1.0 - (1.0 + mu_val) / j) * weights[j-1]
    
    for k in range(num_pts):
        w_sum = 0.0
        for j in range(k + 1):
            w_sum += weights[j] * error_array[k - j]
        deriv[k] = w_sum / (sample_time ** mu_val)
    return list(deriv)

# Execute simulation
data_store = {}
for mu, param in configs.items():
    d_mu = compute_gl_derivative(e, mu, Ts)
    force = np.zeros(N)
    for k in range(N):
        if x[k] > x_wall:
            force[k] = (K * e[k]) + (param['B'] * d_mu[k])
        else:
            force[k] = 0.0
    data_store[mu] = {'F': force, 'd_mu': d_mu}

# --- GRAPH GENERATION ENGINE ---
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
fig, axs = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Characterization Profiles: Parallel Fractional Kelvin-Voigt Model', fontsize=14, fontweight='bold')

# Graph 1: Force vs Position (The Critical Map)
ax1 = axs[0, 0]
for mu, param in configs.items():
    ax1.plot(x, data_store[mu]['F'], label=f'μ = {mu} (B = {param["B"]})', color=param['color'], linestyle=param['style'], linewidth=2)
ax1.axvline(x=x_wall, color='r', linestyle=':', label='Wall Horizon (x=400)')
ax1.set_title('Graph 1: Force vs Position', fontweight='bold')
ax1.set_xlabel('Position (Encoder Counts)'); ax1.set_ylabel('Rendered Force (PWM Duty)')
ax1.legend(); ax1.grid(True)

# Graph 2: Force vs Time
ax2 = axs[0, 1]
for mu, param in configs.items():
    ax2.plot(t, data_store[mu]['F'], label=f'μ = {mu}', color=param['color'], linestyle=param['style'], linewidth=2)
ax2.set_title('Graph 2: Force vs Time', fontweight='bold')
ax2.set_xlabel('Time (Seconds)'); ax2.set_ylabel('Force')
ax2.legend(); ax2.grid(True)

# Graph 3: Fractional Derivative vs Time
ax3 = axs[1, 0]
for mu, param in configs.items():
    ax3.plot(t, data_store[mu]['d_mu'], label=f'μ = {mu}', color=param['color'], linestyle=param['style'], linewidth=2)
ax3.set_title('Graph 3: Fractional Derivative vs Time', fontweight='bold')
ax3.set_xlabel('Time (Seconds)'); ax3.set_ylabel('D^μ e(t)')
ax3.legend(); ax3.grid(True)

# Graph 4: Hysteresis Loop (Energy Dissipation Footprint)
ax4 = axs[1, 1]
# Focus on μ=0.5 to show clean organic hysteresis memory vs pure viscosity
ax4.plot(x, data_store[0.5]['F'], color=configs[0.5]['color'], linewidth=2.5, label='μ = 0.5 (Viscoelastic Memory)')
ax4.plot(x, data_store[1.0]['F'], color=configs[1.0]['color'], linewidth=1.5, linestyle='--', label='μ = 1.0 (Pure Damping)')
# Add directional arrows manually to show loading vs unloading paths
idx_mid = len(t) // 4
ax4.annotate('', xy=(x[idx_mid], data_store[0.5]['F'][idx_mid]), xytext=(x[idx_mid-5], data_store[0.5]['F'][idx_mid-5]),
             arrowprops=dict(arrowstyle="->", color='black', lw=2))
ax4.set_title('Graph 4: Hysteresis Loop (Energy Dissipation)', fontweight='bold')
ax4.set_xlabel('Position (Encoder Counts)'); ax4.set_ylabel('Force')
ax4.legend(); ax4.grid(True)

plt.tight_layout()
plt.show()