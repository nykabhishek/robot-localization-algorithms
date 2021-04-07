## Extended Kalman Filter for Lidar and Radar Sensor Fusion

An extended Kalman Filter implementation in `jupyter-notebook` for fusing lidar and radar sensor measurements for a **Constant Turn Rate and Velocity Vehicle Model (CTRV)**.

**Data is received from two 'noisy' sensors:**
- A LIDAR sensor that measures the position `(x, y)` in cartesian-coordinates.
- A RADAR that measures the position ($\rho$) , relative velocity ($\dot{\rho}$) and heading angle ($\phi$) in polar coordinates. 

**The Extended Kalman filter predicts the position ($p_x$, $p_y$), velocity ($v_x$, $v_y$), yaw angle ($\psi$) and yaw rate ($\dot{\psi}$)**

```
[L(for lidar)] [px] [py] [t] [r_px] [r_py] [r_vx] [r_vy] [yaw] [dyaw]
[R(for radar)] [rho] [phi] [drho] [t] [r_px] [r_py] [r_vx] [r_vy] [yaw] [dyaw]

Where:
(px, py) - measurements by the lidar
(rho, phi, dho) - measurements by the radar in polar coordinates
(t) - timestamp in unix/epoch time the measurements were taken
(r_px, r_py, r_vx, r_vy, yaw, dyaw) - the real ground truth state of the system

Example:
L 8.45  0.25  1477010443349642  8.45  0.25  -3.00027  0 0.12 0.3
R 8.60363 0.0290616 -2.99903  1477010443399637  8.6 0.25  -3.00029  0 0.14 0.35
```
