import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# FO-SLS PARAMETERS
# -----------------------------
K0 = 2.89
K1 = 5.70
B1 = 5.89
alpha = 0.5

Ts = 0.001          # 1 ms
N = 101             # Memory length
sim_time = 5.0      # seconds

samples = int(sim_time / Ts)

# -----------------------------
# GL WEIGHTS
# -----------------------------
weights = np.zeros(N)
weights[0] = 1.0

for j in range(1, N):
    weights[j] = weights[j-1] * (1 - ((alpha + 1) / j))

# -----------------------------
# COEFFICIENTS
# -----------------------------
Ts_alpha = Ts ** alpha

tau_alpha = B1 / K1
B_dyn = B1 * (1 + K0 / K1)

denom = 1 + tau_alpha / Ts_alpha

Ce0 = (K0 + B_dyn / Ts_alpha) / denom
Ceh = (B_dyn / Ts_alpha) / denom
Cfh = (tau_alpha / Ts_alpha) / denom

print("Coefficients")
print(Ce0, Ceh, Cfh)

# -----------------------------
# Buffers
# -----------------------------
errorHistory = np.zeros(N)
forceHistory = np.zeros(N)

bufferIndex = 0

force = []
time = []

# -----------------------------
# STEP INPUT
# -----------------------------
# Constant penetration after t=0

penetration = 20.0

for k in range(samples):

    e = penetration

    sum_e = 0
    sum_f = 0

    for j in range(1, N):

        idx = (bufferIndex - j) % N

        sum_e += weights[j] * errorHistory[idx]
        sum_f += weights[j] * forceHistory[idx]

    F = -(Ce0 * e + Ceh * sum_e - Cfh * sum_f)

    errorHistory[bufferIndex] = e
    forceHistory[bufferIndex] = F

    bufferIndex = (bufferIndex + 1) % N

    force.append(F)
    time.append(k * Ts)

# -----------------------------
# Plot
# -----------------------------
plt.figure(figsize=(9,5))
plt.plot(time, force)
plt.grid(True)
plt.xlabel("Time (s)")
plt.ylabel("Force")
plt.title("FO-SLS Step Response")
plt.show()