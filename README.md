# Simulation
<img width="1275" height="875" alt="Screenshot from 2026-06-14 01-23-51" src="https://github.com/user-attachments/assets/e6e4678f-88f2-4213-b740-360c72491137" />


# Kinematic Model

## Pan Rotation

The pan mechanism rotates the upper assembly around the global Z-axis.

Rotation matrix around the Z-axis:

```math
R_z(\theta)=
\begin{bmatrix}
\cos(\theta) & -\sin(\theta) & 0 \\
\sin(\theta) & \cos(\theta) & 0 \\
0 & 0 & 1
\end{bmatrix}
```

Point transformation around the pan pivot:

```math
P' = R_z(\theta)\,(P - P_{pan}) + P_{pan}
```

Where:

```math
P =
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
```

and

```math
P_{pan} =
\begin{bmatrix}
x_p \\
y_p \\
z_p
\end{bmatrix}
```

---

## Tilt Rotation

The tilt mechanism rotates the sensor assembly around the local Y-axis.

Rotation matrix around the Y-axis:

```math
R_y(\phi)=
\begin{bmatrix}
\cos(\phi) & 0 & \sin(\phi) \\
0 & 1 & 0 \\
-\sin(\phi) & 0 & \cos(\phi)
\end{bmatrix}
```

Point transformation around the tilt pivot:

```math
P'' = R_y(\phi)\,(P - P_{tilt}) + P_{tilt}
```

Where:

```math
P_{tilt} =
\begin{bmatrix}
x_t \\
y_t \\
z_t
\end{bmatrix}
```

---

## Hierarchical Kinematic Transformation

The system applies hierarchical transformations in two stages.

### Stage 1 — Tilt Rotation

```math
P_1 = R_y(\phi)\,(P - P_{tilt}) + P_{tilt}
```

### Stage 2 — Pan Rotation

```math
P_2 = R_z(\theta)\,(P_1 - P_{pan}) + P_{pan}
```

Substituting (P_1):

```math
P_2 =
R_z(\theta)
\left[
R_y(\phi)(P - P_{tilt})
+
P_{tilt}
-
P_{pan}
\right]
+
P_{pan}
```

This equation represents the complete forward kinematic model of the dual-axis tracker.

---

# Homogeneous Transformation Model

Homogeneous coordinate representation:

```math
P_h =
\begin{bmatrix}
x \\
y \\
z \\
1
\end{bmatrix}
```

---

## Pan Transformation Matrix

```math
T_{pan} =
\begin{bmatrix}
\cos(\theta) & -\sin(\theta) & 0 & x_p \\
\sin(\theta) & \cos(\theta) & 0 & y_p \\
0 & 0 & 1 & z_p \\
0 & 0 & 0 & 1
\end{bmatrix}
```

---

## Tilt Transformation Matrix

```math
T_{tilt} =
\begin{bmatrix}
\cos(\phi) & 0 & \sin(\phi) & x_t \\
0 & 1 & 0 & y_t \\
-\sin(\phi) & 0 & \cos(\phi) & z_t \\
0 & 0 & 0 & 1
\end{bmatrix}
```

---

## Overall Transformation

```math
T_{total} = T_{pan}\,T_{tilt}
```

Final transformed point:

```math
P_{out} = T_{total}\,P_h
```

---

# Angular Velocity Model

Pan angular velocity vector:

```math
\omega_{pan} =
\begin{bmatrix}
0 \\
0 \\
\dot{\theta}
\end{bmatrix}
```

Tilt angular velocity vector:

```math
\omega_{tilt} =
\begin{bmatrix}
0 \\
\dot{\phi} \\
0
\end{bmatrix}
```

Total angular velocity:

```math
\omega_{total}
=
\omega_{pan}
+
R_z(\theta)\,\omega_{tilt}
```

Expanded form:

```math
\omega_{total} =
\begin{bmatrix}
-\dot{\phi}\sin(\theta) \\
\dot{\phi}\cos(\theta) \\
\dot{\theta}
\end{bmatrix}
```

---

# Angular Acceleration Model

Angular acceleration vector:

```math
\alpha_{total}
=
\frac{d}{dt}
\left(
\omega_{total}
\right)
```

Expanded form:

```math
\alpha_{total} =
\begin{bmatrix}
-\ddot{\phi}\sin(\theta)
-
\dot{\phi}\dot{\theta}\cos(\theta)
\\
\ddot{\phi}\cos(\theta)
-
\dot{\phi}\dot{\theta}\sin(\theta)
\\
\ddot{\theta}
\end{bmatrix}
```

---

# Automatic Search Motion

Target update equations:

```math
\theta_{next}
=
\theta_{current}
+
\Delta\theta
```

```math
\phi_{next}
=
\phi_{current}
+
\Delta\phi
```

Incremental motion equations:

```math
\Delta\theta
=
k_1
(
\theta_{target}
-
\theta_{current}
)
```

```math
\Delta\phi
=
k_2
(
\phi_{target}
-
\phi_{current}
)
```

Where:

- $k_1$ is the pan interpolation coefficient  
- $k_2$ is the tilt interpolation coefficient
