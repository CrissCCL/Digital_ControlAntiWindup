import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# =========================================================
# Discretization of First-Order Identified Model
# =========================================================
Ts = 0.1
gp  = ctrl.tf([20], [50, 1])                 # First-order plant
gpd = ctrl.c2d(gp, Ts, method='zoh')         # Discrete plant

# =========================================================
# PI controller parameters
# =========================================================
Kp = 0.8
Ti = 9.0

# =========================================================
# Extract discrete model coefficients (robust)
# MATLAB: y[k] = num(2)*u - den(2)*y[k-1]
# =========================================================
num, den = ctrl.tfdata(gpd)
num = np.asarray(num, dtype=float).squeeze()
den = np.asarray(den, dtype=float).squeeze()

num = np.ravel(num)
den = np.ravel(den)

# Normalize denominator
if den[0] != 1.0:
    num = num / den[0]
    den = den / den[0]

# Ensure numerator has delay term
if num.size == 1:
    num = np.array([0.0, float(num[0])])

b1 = float(num[1])
a1 = float(den[1])

print("Using discrete plant:")
print("y[k] = b1*u[k-1] - a1*y[k-1]")
print("b1 =", b1, " a1 =", a1)

# =========================================================
# Closed-loop simulation
# =========================================================
t = np.arange(0, 60 + Ts, Ts)
M = 400
Ref = M * np.ones(len(t))

y = np.zeros(len(t))
Usim = np.zeros(len(t))

# States
y1 = 0.0     # previous output
I  = 0.0     # integrator state
u  = 0.0     # control signal

for k in range(len(t)):
    # ---- PLANT ----
    y[k] = b1 * u - a1 * y1

    # ---- FEEDBACK (NO ANTI-WINDUP) ----
    e = Ref[k] - y[k]
    I = I + Ts * e
    u = Kp * e + (Kp / Ti) * I

    # ---- SATURATION ----
    if u > 100:
        u = 100
    if u < 0:
        u = 0

    # ---- UPDATE STATES ----
    y1 = y[k]
    Usim[k] = u

# =========================================================
# PLOTS
# =========================================================
plt.figure(figsize=(8, 6))

plt.subplot(2, 1, 1)
plt.plot(t, y, '+', markersize=4, label='Simulation')
plt.plot(t, Ref, '--', label='Reference')
plt.xlabel('Time [s]')
plt.ylabel('Response')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t, Usim, '+', markersize=4)
plt.xlabel('Time [s]')
plt.ylabel('Control signal')
plt.grid(True)

plt.tight_layout()
plt.show()
