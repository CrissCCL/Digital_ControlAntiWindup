# 🧪 Digital Control Simulation — First-Order System + Anti-Windup PI

This repository provides a **tutorial-oriented simulation** of a **digital PI control loop** applied to a **first-order system identified via non-parametric methods**.  
The simulation includes **actuator saturation** and an **anti-windup algorithm using conditional integration**, allowing the user to visualize the same discrete behavior expected on a **microcontroller** (Arduino, Teensy, ESP32, etc.).

> ⚠️ **Note:** This tutorial is for **educational purposes only**, focusing on understanding discrete-time digital control, actuator limits, and anti-windup strategies.

## 🎯 Goals

- Model a first-order process identified experimentally (non-parametric fit)  
- Discretize the plant using Zero-Order Hold (ZOH)  
- Implement a **discrete PI controller in incremental form**  
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

```matlab
y(k) = num(2)*u1 - den(2)*y1;
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

```matlab
if (u>=100 && e>0) || (u<=0 && e<0)
    I = I;       % Freeze integrator
else
    I = I + Ts*e;
end
u = Kp*e + Kp/Ti*I;
```

## 🔒 Actuator Saturation

To emulate **real actuator limits** in microcontrollers, the control output is **constrained**:

$$
U_{\min} \le u(k) \le U_{\max}
$$

This ensures that the controller output never exceeds the physical limits of the actuator (e.g., PWM 0–100%).

In MATLAB/Simulink:

```matlab
if u > 100
    u=100;
end
if u< 0
    u=0;
end
```

Below are example plots generated with the script:

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/2e69e4aa-a990-4001-acc3-faddbced0f4b" alt="Response Without saturation" width="500"><br>
      <sub>Response Without saturation </sub>
    </td>
    <td align="center">
        <img  src="https://github.com/user-attachments/assets/e0cbebac-2882-4911-8a68-9841a0ec4633" alt="Response With anti windup" width="550"><br>
      <sub>Response saturation With anti windup </sub>
    </td>
  </tr>
</table>

Response saturation Without anti windup:
<p align="center">
  <img src="https://github.com/user-attachments/assets/8a372c35-61a1-4232-9fa8-a367dd3a57b7" alt="Complete Prototype Setup - Version 3" width="400">
</p>


## 📜 License
MIT License

