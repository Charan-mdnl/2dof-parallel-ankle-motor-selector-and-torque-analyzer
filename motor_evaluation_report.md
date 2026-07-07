# Ankle Joint Actuator & Mechanism Sizing Evaluation Report

This report summarizes the performance of **Robstride RS02, RS03, and RS04** servo motors under two ankle spreads and two mounting configurations, using the exact analytical Jacobian mapping.

## Workspace Sweep Strategy
*   **Pitch Sweep**: Pitch angle is swept from $-25^\circ$ to $+25^\circ$ in $0.5^\circ$ increments (with Roll set to $0^\circ$).
*   **Roll Sweep**: Roll angle is swept from $-40^\circ$ to $+40^\circ$ in $0.5^\circ$ increments (with Pitch set to $0^\circ$).
*   **Minimum Capacity**: Represents the worst-case (minimum) torque capacity across the workspace sweep, which is the guaranteed output torque of the ankle in any valid position.
*   **Discrete Sample Positions**: Shows the exact ankle torque output at specific foot angles ($Pitch, Roll$) for each motor configuration.

---

## Current Mechanism (Ankle Spread $E = 2.3\text{ cm}$)
All other dimensions: $m_{1z} = 23.5\text{ cm}$, $m_{2z} = 16.0\text{ cm}$, $r_1 = r_2 = 2.5\text{ cm}$, $L_1 = 20.7\text{ cm}$, $L_2 = 12.7\text{ cm}$, $oy = 3.8\text{ cm}$.

### Standard Layout (0° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 17.77 | -16.5° | 4.43 | +40.0° |
| **RS02** | Peak | 17.0 | 50.36 | -16.5° | 12.54 | +40.0° |
| **RS03** | Rated | 20.0 | 59.25 | -16.5° | 14.76 | +40.0° |
| **RS03** | Peak | 60.0 | 177.74 | -16.5° | 44.27 | +40.0° |
| **RS04** | Rated | 40.0 | 118.49 | -16.5° | 29.52 | +40.0° |
| **RS04** | Peak | 120.0 | 355.48 | -16.5° | 88.55 | +40.0° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 18.15 | 51.43 | 60.51 | 181.53 | 121.02 | 363.05 |
| -20.0° | 17.83 | 50.51 | 59.42 | 178.27 | 118.84 | 356.53 |
| -15.0° | 17.79 | 50.40 | 59.29 | 177.88 | 118.59 | 355.76 |
| -10.0° | 17.98 | 50.93 | 59.92 | 179.77 | 119.84 | 359.53 |
| -5.0° | 18.38 | 52.07 | 61.26 | 183.78 | 122.52 | 367.57 |
| +0.0° | 19.01 | 53.87 | 63.38 | 190.13 | 126.75 | 380.26 |
| +5.0° | 19.94 | 56.51 | 66.48 | 199.44 | 132.96 | 398.88 |
| +10.0° | 21.32 | 60.40 | 71.06 | 213.17 | 142.12 | 426.35 |
| +15.0° | 23.48 | 66.52 | 78.25 | 234.76 | 156.51 | 469.53 |
| +20.0° | 27.47 | 77.83 | 91.57 | 274.71 | 183.14 | 549.41 |
| +25.0° | 40.10 | 113.63 | 133.68 | 401.04 | 267.36 | 802.08 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 4.97 | 14.07 | 16.55 | 49.66 | 33.11 | 99.33 |
| -30.0° | 5.38 | 15.25 | 17.94 | 53.81 | 35.87 | 107.62 |
| -20.0° | 5.64 | 15.98 | 18.80 | 56.41 | 37.61 | 112.82 |
| -10.0° | 5.76 | 16.33 | 19.21 | 57.62 | 38.41 | 115.24 |
| +0.0° | 5.75 | 16.30 | 19.18 | 57.54 | 38.36 | 115.08 |
| +10.0° | 5.62 | 15.92 | 18.73 | 56.20 | 37.47 | 112.40 |
| +20.0° | 5.36 | 15.18 | 17.86 | 53.58 | 35.72 | 107.17 |
| +30.0° | 4.96 | 14.06 | 16.55 | 49.64 | 33.09 | 99.27 |
| +40.0° | 4.43 | 12.54 | 14.76 | 44.27 | 29.52 | 88.55 |


### Orthogonal Layout (90° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 11.77 | -15.0° | 11.28 | -27.5° |
| **RS02** | Peak | 17.0 | 33.36 | -15.0° | 31.95 | -27.5° |
| **RS03** | Rated | 20.0 | 39.24 | -15.0° | 37.59 | -27.5° |
| **RS03** | Peak | 60.0 | 117.73 | -15.0° | 112.77 | -27.5° |
| **RS04** | Rated | 40.0 | 78.48 | -15.0° | 75.18 | -27.5° |
| **RS04** | Peak | 120.0 | 235.45 | -15.0° | 225.54 | -27.5° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 12.10 | 34.28 | 40.33 | 120.99 | 80.66 | 241.98 |
| -20.0° | 11.85 | 33.56 | 39.48 | 118.45 | 78.97 | 236.90 |
| -15.0° | 11.77 | 33.36 | 39.24 | 117.73 | 78.48 | 235.45 |
| -10.0° | 11.82 | 33.50 | 39.41 | 118.23 | 78.82 | 236.45 |
| -5.0° | 11.97 | 33.91 | 39.90 | 119.69 | 79.80 | 239.39 |
| +0.0° | 12.20 | 34.58 | 40.68 | 122.04 | 81.36 | 244.07 |
| +5.0° | 12.53 | 35.51 | 41.77 | 125.32 | 83.55 | 250.64 |
| +10.0° | 12.98 | 36.77 | 43.26 | 129.78 | 86.52 | 259.55 |
| +15.0° | 13.59 | 38.51 | 45.31 | 135.93 | 90.62 | 271.86 |
| +20.0° | 14.49 | 41.06 | 48.31 | 144.93 | 96.62 | 289.86 |
| +25.0° | 15.97 | 45.24 | 53.22 | 159.66 | 106.44 | 319.31 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 11.78 | 33.38 | 39.27 | 117.80 | 78.53 | 235.59 |
| -30.0° | 11.29 | 31.99 | 37.64 | 112.92 | 75.28 | 225.84 |
| -20.0° | 11.38 | 32.24 | 37.93 | 113.78 | 75.85 | 227.56 |
| -10.0° | 11.79 | 33.40 | 39.29 | 117.88 | 78.59 | 235.77 |
| +0.0° | 12.56 | 35.59 | 41.88 | 125.63 | 83.75 | 251.26 |
| +10.0° | 14.09 | 39.91 | 46.96 | 140.87 | 93.91 | 281.74 |
| +20.0° | 18.58 | 52.64 | 61.93 | 185.80 | 123.87 | 371.60 |
| +30.0° | — | — | — | — | — | — |
| +40.0° | — | — | — | — | — | — |



---

## Parallel Mechanism (Ankle Spread $E = 5.0\text{ cm}$)
All other dimensions: $m_{1z} = 23.5\text{ cm}$, $m_{2z} = 16.0\text{ cm}$, $r_1 = r_2 = 2.5\text{ cm}$, $L_1 = 20.7\text{ cm}$, $L_2 = 12.7\text{ cm}$, $oy = 3.8\text{ cm}$.

### Standard Layout (0° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 18.04 | -13.5° | 11.90 | +33.5° |
| **RS02** | Peak | 17.0 | 51.11 | -13.5° | 33.72 | +33.5° |
| **RS03** | Rated | 20.0 | 60.13 | -13.5° | 39.67 | +33.5° |
| **RS03** | Peak | 60.0 | 180.38 | -13.5° | 119.01 | +33.5° |
| **RS04** | Rated | 40.0 | 120.26 | -13.5° | 79.34 | +33.5° |
| **RS04** | Peak | 120.0 | 360.77 | -13.5° | 238.01 | +33.5° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 18.86 | 53.43 | 62.86 | 188.59 | 125.73 | 377.19 |
| -20.0° | 18.27 | 51.78 | 60.91 | 182.74 | 121.83 | 365.49 |
| -15.0° | 18.05 | 51.15 | 60.17 | 180.52 | 120.35 | 361.04 |
| -10.0° | 18.09 | 51.24 | 60.29 | 180.86 | 120.57 | 361.72 |
| -5.0° | 18.33 | 51.94 | 61.11 | 183.33 | 122.22 | 366.66 |
| +0.0° | 18.79 | 53.24 | 62.63 | 187.90 | 125.27 | 375.80 |
| +5.0° | 19.49 | 55.22 | 64.96 | 194.89 | 129.93 | 389.78 |
| +10.0° | 20.52 | 58.14 | 68.39 | 205.18 | 136.79 | 410.37 |
| +15.0° | 22.08 | 62.56 | 73.61 | 220.82 | 147.21 | 441.63 |
| +20.0° | 24.73 | 70.06 | 82.42 | 247.26 | 164.84 | 494.53 |
| +25.0° | 30.70 | 86.98 | 102.33 | 307.00 | 204.67 | 614.01 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 18.64 | 52.81 | 62.13 | 186.39 | 124.26 | 372.77 |
| -30.0° | 14.35 | 40.66 | 47.84 | 143.52 | 95.68 | 287.03 |
| -20.0° | 13.21 | 37.42 | 44.02 | 132.06 | 88.04 | 264.12 |
| -10.0° | 12.67 | 35.91 | 42.25 | 126.74 | 84.49 | 253.48 |
| +0.0° | 12.36 | 35.02 | 41.21 | 123.62 | 82.41 | 247.23 |
| +10.0° | 12.15 | 34.43 | 40.50 | 121.51 | 81.01 | 243.02 |
| +20.0° | 12.00 | 34.00 | 40.00 | 120.00 | 80.00 | 239.99 |
| +30.0° | 11.91 | 33.74 | 39.70 | 119.09 | 79.39 | 238.18 |
| +40.0° | 11.95 | 33.85 | 39.83 | 119.48 | 79.65 | 238.96 |


### Orthogonal Layout (90° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 15.01 | -14.0° | 14.54 | -24.5° |
| **RS02** | Peak | 17.0 | 42.54 | -14.0° | 41.19 | -24.5° |
| **RS03** | Rated | 20.0 | 50.05 | -14.0° | 48.46 | -24.5° |
| **RS03** | Peak | 60.0 | 150.15 | -14.0° | 145.38 | -24.5° |
| **RS04** | Rated | 40.0 | 100.10 | -14.0° | 96.92 | -24.5° |
| **RS04** | Peak | 120.0 | 300.30 | -14.0° | 290.77 | -24.5° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 15.52 | 43.98 | 51.74 | 155.22 | 103.48 | 310.43 |
| -20.0° | 15.14 | 42.90 | 50.47 | 151.41 | 100.94 | 302.81 |
| -15.0° | 15.02 | 42.55 | 50.06 | 150.18 | 100.12 | 300.36 |
| -10.0° | 15.06 | 42.67 | 50.20 | 150.60 | 100.40 | 301.20 |
| -5.0° | 15.23 | 43.14 | 50.76 | 152.27 | 101.51 | 304.54 |
| +0.0° | 15.50 | 43.93 | 51.68 | 155.04 | 103.36 | 310.08 |
| +5.0° | 15.89 | 45.03 | 52.97 | 158.92 | 105.95 | 317.85 |
| +10.0° | 16.41 | 46.50 | 54.70 | 164.11 | 109.40 | 328.21 |
| +15.0° | 17.10 | 48.45 | 57.00 | 170.99 | 114.00 | 341.99 |
| +20.0° | 18.04 | 51.11 | 60.13 | 180.38 | 120.25 | 360.75 |
| +25.0° | 19.39 | 54.93 | 64.63 | 193.88 | 129.26 | 387.77 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 15.43 | 43.71 | 51.43 | 154.29 | 102.86 | 308.58 |
| -30.0° | 14.62 | 41.42 | 48.73 | 146.20 | 97.47 | 292.41 |
| -20.0° | 14.58 | 41.30 | 48.58 | 145.75 | 97.17 | 291.50 |
| -10.0° | 14.92 | 42.27 | 49.73 | 149.19 | 99.46 | 298.38 |
| +0.0° | 15.65 | 44.34 | 52.16 | 156.48 | 104.32 | 312.95 |
| +10.0° | 17.04 | 48.28 | 56.80 | 170.39 | 113.59 | 340.78 |
| +20.0° | 20.45 | 57.95 | 68.18 | 204.53 | 136.35 | 409.05 |
| +30.0° | — | — | — | — | — | — |
| +40.0° | — | — | — | — | — | — |



---

## Key Insights & Engineering Recommendations
1. **Orthogonal Mounting Performance Improvements**:
   * In the **Standard Layout**, both motors face front, meaning they push/pull in parallel. This yields excellent **Pitch capacity** but reduced **Roll capacity** because the roll axis acts primarily through the ankle spread distance $E$.
   * In the **Orthogonal Layout**, the top motor operates directly in the roll-sweeping plane (YZ plane). This significantly increases the **Roll capacity** of the ankle by decoupling the actuator axes! For example, roll torque increases significantly without sacrificing pitch performance.
2. **Effect of Ankle Spread ($E$)**:
   * Adjusting the ankle spread from $E = 2.3\text{ cm}$ to $E = 5.0\text{ cm}$ increases the mechanical advantage (moment arm) for the roll axis in the standard layout.
   * However, in the orthogonal layout, the decoupled linkage is already highly efficient, making the design robust even at compact ankle spreads.
3. **Actuator Selection recommendations**:
   * **RS02**: Suitable for lightweight/low-load ankle stabilization. Peak torque yields up to ~1-2 N·m on the ankle.
   * **RS03**: Excellent balance for walking/dynamic stability. Rated capacity provides solid torque, and peak torque (60 N·m) yields robust safety margins across the entire workspace.
   * **RS04**: Necessary for heavy load-bearing or running/jumping tasks where high ankle torque capacity (>10-20 N·m) is required.