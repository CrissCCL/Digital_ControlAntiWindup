# 🧪 Digital Control Simulation — First-Order System + Anti-Windup PI

![MATLAB](https://img.shields.io/badge/MATLAB-Simulation-blue)
![Python](https://img.shields.io/badge/Python-Analysis-green)
![Control](https://img.shields.io/badge/Control-Anti--Windup-orange)
![PID](https://img.shields.io/badge/PID-Implementation-green)
![Robustness](https://img.shields.io/badge/Robustness-Actuator%20Limits-lightgrey)
![License](https://img.shields.io/badge/License-MIT-lightgrey)


This repository provides a **tutorial-oriented simulation** of a **digital PI control loop** applied to a **first-order system identified via non-parametric methods**.  
The simulation includes **actuator saturation** and an **anti-windup algorithm using conditional integration**, allowing the user to visualize the same discrete behavior expected on a **microcontroller** (Arduino, Teensy, ESP32, etc.).

> ⚠️ **Note:** This tutorial is for **educational purposes only**, focusing on understanding discrete-time digital control, actuator limits, and anti-windup strategies.

## 🎯 Goals

- Model a first-order process identified experimentally (non-parametric fit)  
- Discretize the plant using Zero-Order Hold (ZOH)  
- Implement a **discrete PI controller in positional form**  
- Include **saturation limits** to emulate real actuator constraints  
- Add **anti-windup** via **conditional integration**  
- Observe reference tracking and control signal behavior  

## 🧩 System Model

A first-order model without delay was identified experimentally from step-response data:

$$
G_p(s) = \frac{K}{\tau s + 1}
$$

Example parameters used in this tutorial:

$$
G_p(s)= \frac{20}{50s + 1}, \quad T_s = 0.1 \text{ s}
$$

Discretized with **Zero-Order Hold**.  

Discrete-time implementation:

Matlab
```Matlab
y(k) = num(2)*u1 - den(2)*y1;
```
Python
```Python
y[k] = b1 * u - a1 * y1
```


## 🧠 Why the Incremental PI Form Does Not Need Anti-Windup

When implementing a PI in **incremental form**, the integral action appears **only as a difference between errors** and **is not accumulated explicitly**. The control law is:

$$
u(k)=u(k-1) + K_0e(k) + K_1 e(k-1)
$$

This form **does not integrate the error in an accumulating memory**, but instead adjusts the control from the last value.  
Because the controller does **not store the integral sum**, once saturation is applied:

- The incremental update is naturally “cut off”
- No further accumulation occurs beyond limits
- There is no wind-up phenomenon to correct

Therefore, in incremental PI controllers, explicit anti-windup logic is **not required** — saturation alone is sufficient.


## 🎛️ Positional PI With Conditional Anti-Windup

In contrast, the **positional PI** form computes:

$$
u(k)=K_p e(k)+\frac{K_p}{ T_i} \cdot I(k)
$$

Where the term  

$$
I(k)=\sum e(k) T_s
$$  

**does accumulate over time**. If the actuator saturates, the integral would continue growing and later produce large overshoot when saturation exits — this is the classical **integrator wind-up**.

To prevent it, **conditional integration** is used:

- Integrate only when the control is not saturated, **or**
- When the error would help the controller exit saturation

Matlab
```Matlab
if (u>=100 && e>0) || (u<=0 && e<0)
    I = I;       % Freeze integrator
else
    I = I + Ts*e;
end
u = Kp*e + Kp/Ti*I;
```
Python
```Python
if (u >= 100 and e > 0) or (u <= 0 and e < 0):
    I = I
else:
    I = I + Ts * e

u = Kp * e + (Kp / Ti) * I
```

## 🔒 Actuator Saturation

To emulate **real actuator limits** in microcontrollers, the control output is **constrained**:

$$
U_{\min} \le u(k) \le U_{\max}
$$

This ensures that the controller output never exceeds the physical limits of the actuator (e.g., PWM 0–100%).

Matlab
```Matlab
if u > 100
    u=100;
end
if u< 0
    u=0;
end
```
Python
```Python
if u > 100:
    u = 100
if u < 0:
    u = 0
```

Below are example plots generated with the script:

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/3a9de997-1b47-498b-bae6-3b95c35f6383" alt="Response Without saturation" width="500"><br>
      <sub>Response Without saturation </sub>
    </td>
    <td align="center">
        <img  src="https://github.com/user-attachments/assets/ffb78960-307a-4c58-8975-2750e5639931" alt="Response With anti windup" width="550"><br>
      <sub>Response saturation With antiwindup </sub>
    </td>
  </tr>
</table>

Response saturation Without antiwindup:
<p align="center">
  <img  src="https://github.com/user-attachments/assets/924e9d41-2f22-4565-9cbd-169c3f1cdb28" alt="Response saturation Without antiwindup " width="400">
</p>

## 🤝 Support projects
 Support me on Patreon [https://www.patreon.com/c/CrissCCL](https://www.patreon.com/c/CrissCCL)

## 📜 License
MIT License
