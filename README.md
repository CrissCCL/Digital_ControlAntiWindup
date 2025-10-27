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

---

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

## ⚙️ Digital PI Controller (Incremental Form)

The PI control law in **incremental form** is expressed as:

$$
u(k) = u(k-1) + K_p [ e(k) - e(k-1) ] + \frac{K_p T_s}{2 T_i}[ e(k) + e(k-1)]
$$
Where Trapezoidal sum part is,
$$
\Delta I = e(k) + e(k-1)
$$

$$
u(k) = u(k-1) + K_p [ e(k) - e(k-1) ] + \frac{K_p T_s}{2 T_i}\Delta I
$$


With tuning parameters derived from:
- Proportional gain: $$K_p$$
- Integral time: $$T_i$$
- Sampling time: $$T_s$$

## 🔒 Anti-Windup via Conditional Integration

To prevent integrator wind-up when the actuator saturates, a **conditional integration scheme** is applied:

- Only integrate the error when the control output is **not saturated** or when the error would **reduce the saturation**.
- Otherwise, the integral term is **frozen**.

Mathematically:

$$
\text{if } (u \ge U_{\max} \text{ and } e>0) \text{ or } (u \le U_{\min} \text{ and } e<0), \quad \text{then } I = 0
$$

$$
\text{else } I = \Delta I
$$

Where:  
- $$u$$ is the controller output  
- $$e(k)$$ is the current error  
- $$I$$ is the integral sum term  

```matlab
    deltaI=(error+error1);
    if u>=100 && error>0 || u<=0 && error<0
        Intsum=0;
    else
        Intsum=deltaI;
    end
     u  =u1+ Kp*error-Kp*error1+Kp*Ts/(Ti*2)*Intsum;
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
        <img src="https://github.com/user-attachments/assets/d5711ae6-3ffa-472c-952a-2195dab0dadf" alt="Response With saturation" width="550"><br>
      <sub>Response With saturation </sub>
    </td>
  </tr>
</table>

## 📜 License
MIT License

