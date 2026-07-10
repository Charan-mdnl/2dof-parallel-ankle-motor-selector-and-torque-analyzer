# 2-DOF Parallel Ankle Motor Selector and Torque Analyzer

An interactive design tool, 3D volumetric visualizer, and kinematic solver for **2-RSS parallel ankle mechanisms** (commonly used in humanoid robot legs). This tool helps roboticists size actuators (e.g., Robstride motors) by simulating joint limits, calculating analytical transmission metrics, and visualizing load capacities in real time.

---

## 🚀 Features
* **Premium Volumetric 3D Visualizer**: Real-time rendering of support column, motor housings, crank arms, pushrods, ball joints, ankle stem, and foot plate using pure HTML5 Canvas with custom painter's depth-sorting.
* **Exact Analytical Jacobian Solver**: Employs the Implicit Function Theorem and analytical rotation derivatives (instead of numerical approximation) for absolute mathematical accuracy.
* **Torque Capacity Analysis**: Estimates maximum Pitch capacity (co-directional motor operation) and maximum Roll capacity (push-pull motor operation).
* **Workspace Conditioning Map**: Renders a live 2D transmission map showing the Manipulability Index ($3 / \kappa$) across the workspace to easily locate near-singular configurations.
* **Multi-Language Core**: Parallel kinematics engine implemented in **JavaScript (Web UI)**, **Python**, and **C++**.

---

## 📐 Kinematic Architecture & Dimensions

The mechanism consists of a fixed column with two servo motors mounted at different heights. The motor crank arms connect to the foot plate via pushrods. The foot plate pivots about a central 2-DOF universal joint.

The default geometry matches the physical robot specs:
* **Motor Centers**: Motor ① (top) $m_{1z} = 23.5\text{ cm}$, Motor ② (bottom) $m_{2z} = 16.0\text{ cm}$
* **Crank Arm Lengths**: $r_1 = r_2 = 2.5\text{ cm}$
* **Connecting Rods**: $L_1 = 20.7\text{ cm}$, $L_2 = 12.7\text{ cm}$ (from ball-joint to ball-joint)
* **Universal Joint Height**: $g = 3.6\text{ cm}$
* **Ankle Spread**: $E = 2.3\text{ cm}$ (distance between the two bottom ball joints)
* **Attachment Offset**: $a_h = 0.0\text{ cm}$ (bottom of both rods align horizontally with the U-joint center)
* **Shaft Y-Offset**: $o_y = 3.8\text{ cm}$ (motors and rod planes are offset by 3.8 cm front-to-back to prevent singularity at neutral)

---

## 🧮 Mathematical Formulation

### 1. Inverse Kinematics (IK)
The position of the crank tip $\mathbf{c}_i$ is parameterized by the crank angle $\theta_i$:
$$\mathbf{c}_1(\theta_1) = \begin{bmatrix} r_1 \sin\theta_1 \\ o_y \\ m_{1z} + r_1 \cos\theta_1 \end{bmatrix}, \quad \mathbf{c}_2(\theta_2) = \begin{bmatrix} -r_2 \sin\theta_2 \\ o_y \\ m_{2z} + r_2 \cos\theta_2 \end{bmatrix}$$

The ankle attachment position $\mathbf{a}_i$ in world space is rotated by the foot rotation matrix $\mathbf{R}(p, r) = \mathbf{R}_x(p)\mathbf{R}_y(r)$ and translated by the universal joint center $\mathbf{a}_c = [0, 0, g]^T$:
$$\mathbf{a}_i(p, r) = \mathbf{R}(p, r) \mathbf{a}_{ib} + \mathbf{a}_c$$

Where $\mathbf{a}_{ib}$ are the local ball joint attachment coordinates on the foot:
$$\mathbf{a}_{1b} = \begin{bmatrix} E/2 \\ o_y \\ a_h \end{bmatrix}, \quad \mathbf{a}_{2b} = \begin{bmatrix} -E/2 \\ o_y \\ a_h \end{bmatrix}$$

By enforcing the rigid rod length constraints:
$$\|\mathbf{c}_i(\theta_i) - \mathbf{a}_i(p, r)\|^2 = L_i^2$$

We expand this into the trigonometric form:
$$A \sin\theta_i + B \cos\theta_i = C$$
And solve analytically for the crank angle $\theta_i$ using the tangent half-angle substitution:
$$\theta_i = \text{atan2}(A, B) \pm \arccos\left(\frac{C}{\sqrt{A^2 + B^2}}\right)$$

---

### 2. Analytical Jacobian (Velocity & Torque Mapping)
Rather than using noisy numerical finite differences, we derive the exact analytical Jacobian using the **Implicit Function Theorem**. Let the constraints be:
$$f_i(\theta_i, p, r) = \|\mathbf{c}_i(\theta_i) - \mathbf{a}_i(p, r)\|^2 - L_i^2 = 0$$

Differentiating yields:
$$\frac{\partial f_i}{\partial \theta_i} d\theta_i + \frac{\partial f_i}{\partial p} dp + \frac{\partial f_i}{\partial r} dr = 0$$

This gives the velocity Jacobian entries ($J_{i,1} = \frac{\partial \theta_i}{\partial p}$ and $J_{i,2} = \frac{\partial \theta_i}{\partial r}$):
$$J = \begin{bmatrix} \frac{(\mathbf{c}_1 - \mathbf{a}_1) \cdot \frac{\partial \mathbf{a}_1}{\partial p}}{(\mathbf{c}_1 - \mathbf{a}_1) \cdot \frac{\partial \mathbf{c}_1}{\partial \theta_1}} & \frac{(\mathbf{c}_1 - \mathbf{a}_1) \cdot \frac{\partial \mathbf{a}_1}{\partial r}}{(\mathbf{c}_1 - \mathbf{a}_1) \cdot \frac{\partial \mathbf{c}_1}{\partial \theta_1}} \\ \frac{(\mathbf{c}_2 - \mathbf{a}_2) \cdot \frac{\partial \mathbf{a}_2}{\partial p}}{(\mathbf{c}_2 - \mathbf{a}_2) \cdot \frac{\partial \mathbf{c}_2}{\partial \theta_2}} & \frac{(\mathbf{c}_2 - \mathbf{a}_2) \cdot \frac{\partial \mathbf{a}_2}{\partial r}}{(\mathbf{c}_2 - \mathbf{a}_2) \cdot \frac{\partial \mathbf{c}_2}{\partial \theta_2}} \end{bmatrix}$$

The partial derivatives of the rotation matrix $\mathbf{R}(p, r) = \mathbf{R}_x(p)\mathbf{R}_y(r)$ are derived as:
$$\frac{\partial \mathbf{R}}{\partial p} = \begin{bmatrix} 0 & 0 & 0 \\ \cos p \sin r & -\sin p & -\cos p \cos r \\ \sin p \sin r & \cos p & -\sin p \cos r \end{bmatrix}$$
$$\frac{\partial \mathbf{R}}{\partial r} = \begin{bmatrix} -\sin r & 0 & \cos r \\ \sin p \cos r & 0 & \sin p \sin r \\ -\cos p \cos r & 0 & -\cos p \sin r \end{bmatrix}$$
Which are used to calculate the exact coordinate velocities $\frac{\partial \mathbf{a}_i}{\partial p} = \frac{\partial \mathbf{R}}{\partial p} \mathbf{a}_{ib}$ and $\frac{\partial \mathbf{a}_i}{\partial r} = \frac{\partial \mathbf{R}}{\partial r} \mathbf{a}_{ib}$.

---

### 3. Torque Capacity Equations
Static torque propagation uses the transpose of the Jacobian ($\boldsymbol{\tau}_{\text{ankle}} = \mathbf{J}^T \boldsymbol{\tau}_{\text{motor}}$). For motor torque limits $\tau_{m1}$ and $\tau_{m2}$:
* **Max Pitch Capacity ($\tau_{p, \max}$)**: Assumes co-directional motor rotation (both push/pull together):
  $$\tau_{p, \max} = |J_{0,0}| \cdot \tau_{m1} + |J_{1,0}| \cdot \tau_{m2}$$
* **Max Roll Capacity ($\tau_{r, \max}$)**: Assumes push-pull opposite motor rotation:
  $$\tau_{r, \max} = |J_{0,1}| \cdot \tau_{m1} + |J_{1,1}| \cdot \tau_{m2}$$

---

## 🛠️ How to Run

### 1. Web UI (Interactive Visualizers & Sizing Dashboard)
To start the local server and open the tools directly from a fresh terminal, run the following commands:
```bash
# 1. Start the HTTP server in the background
python3 -m http.server 8000 &

# 2. Open the 3D Mechanism Kinematic Visualizer (Mechanism Analysis Tool)
xdg-open http://localhost:8000/index_orthogonal.html
# (Alternative: google-chrome http://localhost:8000/index_orthogonal.html)

# 3. Open the Actuator Sizing & Sweep Dashboard (All Torque Tables & Graphs)
xdg-open http://localhost:8000/sizing_report.html
# (Alternative: google-chrome http://localhost:8000/sizing_report.html)
```

*(Note: The legacy standard parallel visualizer is also accessible at `http://localhost:8000/index.html`)*

---

### 2. 📄 Comprehensive Sizing Report PDF
A pre-generated print-ready report is located at:
👉 **[ankle_motor_evaluation_report_comprehensive.pdf](ankle_motor_evaluation_report_comprehensive.pdf)**

To regenerate this PDF from a fresh terminal:
```bash
# 1. Start the HTTP server in the background
python3 -m http.server 8000 &
SERVER_PID=$!

# 2. Wait 2 seconds for server to start, then print to PDF using headless Chrome
sleep 2
google-chrome --headless --disable-gpu --no-sandbox --print-to-pdf=ankle_motor_evaluation_report_comprehensive.pdf http://localhost:8000/comprehensive_report_print.html

# 3. Kill the background server
kill $SERVER_PID
```

---

### 3. Command Line Kinematic Solvers

#### Python Kinematic Solver
```bash
# Standard 2-RSS layout solver
python3 ankle_torque_solver.py

# Orthogonal 90° yaw layout solver
python3 ankle_orthogonal_solver.py
```

#### C++ Kinematic Solver
```bash
# Compile and run standard 2-RSS solver
g++ -O2 -std=c++17 -o ankle_torque_solver ankle_torque_solver.cpp -lm
./ankle_torque_solver

# Compile and run orthogonal solver
g++ -O2 -std=c++17 -o ankle_orthogonal_solver ankle_orthogonal_solver.cpp -lm
./ankle_orthogonal_solver
```

---

## 📄 License
This project is open-source and available under the MIT License.
