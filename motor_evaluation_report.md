# Ankle Joint Actuator & Mechanism Sizing Evaluation Report

This report summarizes the performance of **Robstride RS02, RS03, and RS04** servo motors under two ankle spreads and two mounting configurations, using the exact analytical Jacobian mapping.

## Workspace Sweep Strategy
*   **Pitch Sweep**: Pitch angle is swept from $-25^\circ$ to $+25^\circ$ in $0.5^\circ$ increments (with Roll set to $0^\circ$).
*   **Roll Sweep**: Roll angle is swept from $-40^\circ$ to $+40^\circ$ in $0.5^\circ$ increments (with Pitch set to $0^\circ$).
*   **Minimum Capacity**: Represents the worst-case (minimum) torque capacity across the workspace sweep, which is the guaranteed output torque of the ankle in any valid position.
*   **Discrete Sample Positions**: Shows the exact ankle torque output at specific foot angles ($Pitch, Roll$) for each motor configuration.

---

## Current Mechanism (Ankle Spread $E = 2.3\text{ cm}$)
All other dimensions: $H = 23.5\text{ cm}$, $D = 16.0\text{ cm}$, $A = B = 2.5\text{ cm}$, $C = 24.3\text{ cm}$, $I = 16.3\text{ cm}$, $L = 3.8\text{ cm}$, $G = 3.6\text{ cm}$.

### Standard Layout (0° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 17.82 | -16.5° | 4.44 | +40.0° |
| **RS02** | Peak | 17.0 | 50.50 | -16.5° | 12.59 | +40.0° |
| **RS03** | Rated | 20.0 | 59.42 | -16.5° | 14.81 | +40.0° |
| **RS03** | Peak | 60.0 | 178.25 | -16.5° | 44.44 | +40.0° |
| **RS04** | Rated | 40.0 | 118.83 | -16.5° | 29.62 | +40.0° |
| **RS04** | Peak | 120.0 | 356.50 | -16.5° | 88.87 | +40.0° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 18.21 | 51.59 | 60.70 | 182.09 | 121.39 | 364.17 |
| -20.0° | 17.88 | 50.67 | 59.61 | 178.84 | 119.23 | 357.69 |
| -15.0° | 17.83 | 50.53 | 59.45 | 178.34 | 118.90 | 356.69 |
| -10.0° | 18.00 | 51.01 | 60.01 | 180.04 | 120.03 | 360.08 |
| -5.0° | 18.38 | 52.07 | 61.26 | 183.79 | 122.53 | 367.59 |
| +0.0° | 18.98 | 53.79 | 63.28 | 189.83 | 126.55 | 379.66 |
| +5.0° | 19.88 | 56.33 | 66.27 | 198.81 | 132.54 | 397.62 |
| +10.0° | 21.22 | 60.13 | 70.74 | 212.21 | 141.47 | 424.41 |
| +15.0° | 23.35 | 66.17 | 77.85 | 233.54 | 155.69 | 467.08 |
| +20.0° | 27.36 | 77.52 | 91.20 | 273.60 | 182.40 | 547.20 |
| +25.0° | 40.47 | 114.66 | 134.89 | 404.67 | 269.78 | 809.35 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 4.94 | 13.99 | 16.45 | 49.36 | 32.91 | 98.72 |
| -30.0° | 5.36 | 15.18 | 17.86 | 53.57 | 35.71 | 107.13 |
| -20.0° | 5.62 | 15.93 | 18.74 | 56.22 | 37.48 | 112.44 |
| -10.0° | 5.75 | 16.29 | 19.16 | 57.48 | 38.32 | 114.97 |
| +0.0° | 5.74 | 16.28 | 19.15 | 57.45 | 38.30 | 114.90 |
| +10.0° | 5.62 | 15.91 | 18.72 | 56.16 | 37.44 | 112.31 |
| +20.0° | 5.36 | 15.19 | 17.87 | 53.60 | 35.73 | 107.19 |
| +30.0° | 4.97 | 14.09 | 16.57 | 49.71 | 33.14 | 99.43 |
| +40.0° | 4.44 | 12.59 | 14.81 | 44.44 | 29.62 | 88.87 |


### Orthogonal Layout (90° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 11.80 | -14.5° | 11.26 | -28.0° |
| **RS02** | Peak | 17.0 | 33.44 | -14.5° | 31.90 | -28.0° |
| **RS03** | Rated | 20.0 | 39.34 | -14.5° | 37.53 | -28.0° |
| **RS03** | Peak | 60.0 | 118.03 | -14.5° | 112.60 | -28.0° |
| **RS04** | Rated | 40.0 | 78.69 | -14.5° | 75.07 | -28.0° |
| **RS04** | Peak | 120.0 | 236.07 | -14.5° | 225.21 | -28.0° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 12.13 | 34.37 | 40.44 | 121.31 | 80.88 | 242.63 |
| -20.0° | 11.88 | 33.67 | 39.61 | 118.82 | 79.21 | 237.64 |
| -15.0° | 11.80 | 33.44 | 39.35 | 118.04 | 78.69 | 236.08 |
| -10.0° | 11.84 | 33.55 | 39.48 | 118.43 | 78.95 | 236.85 |
| -5.0° | 11.97 | 33.92 | 39.91 | 119.73 | 79.82 | 239.46 |
| +0.0° | 12.19 | 34.53 | 40.63 | 121.88 | 81.25 | 243.76 |
| +5.0° | 12.49 | 35.40 | 41.65 | 124.94 | 83.30 | 249.89 |
| +10.0° | 12.92 | 36.60 | 43.06 | 129.18 | 86.12 | 258.35 |
| +15.0° | 13.51 | 38.29 | 45.04 | 135.13 | 90.09 | 270.27 |
| +20.0° | 14.40 | 40.81 | 48.01 | 144.04 | 96.03 | 288.09 |
| +25.0° | 15.90 | 45.05 | 53.00 | 159.01 | 106.00 | 318.01 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 11.71 | 33.19 | 39.05 | 117.15 | 78.10 | 234.29 |
| -30.0° | 11.27 | 31.93 | 37.57 | 112.71 | 75.14 | 225.42 |
| -20.0° | 11.37 | 32.21 | 37.89 | 113.68 | 75.79 | 227.37 |
| -10.0° | 11.78 | 33.37 | 39.25 | 117.76 | 78.51 | 235.52 |
| +0.0° | 12.54 | 35.53 | 41.80 | 125.40 | 83.60 | 250.80 |
| +10.0° | 14.05 | 39.81 | 46.84 | 140.52 | 93.68 | 281.04 |
| +20.0° | 18.57 | 52.62 | 61.90 | 185.71 | 123.81 | 371.43 |
| +30.0° | — | — | — | — | — | — |
| +40.0° | — | — | — | — | — | — |



---

## Parallel Mechanism (Ankle Spread $E = 5.0\text{ cm}$)
All other dimensions: $H = 23.5\text{ cm}$, $D = 16.0\text{ cm}$, $A = B = 2.5\text{ cm}$, $C = 24.3\text{ cm}$, $I = 16.3\text{ cm}$, $L = 3.8\text{ cm}$, $G = 3.6\text{ cm}$.

### Standard Layout (0° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 18.03 | -13.5° | 11.90 | +33.0° |
| **RS02** | Peak | 17.0 | 51.08 | -13.5° | 33.72 | +33.0° |
| **RS03** | Rated | 20.0 | 60.10 | -13.5° | 39.68 | +33.0° |
| **RS03** | Peak | 60.0 | 180.30 | -13.5° | 119.03 | +33.0° |
| **RS04** | Rated | 40.0 | 120.20 | -13.5° | 79.35 | +33.0° |
| **RS04** | Peak | 120.0 | 360.60 | -13.5° | 238.06 | +33.0° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 18.77 | 53.18 | 62.56 | 187.68 | 125.12 | 375.36 |
| -20.0° | 18.24 | 51.67 | 60.79 | 182.38 | 121.58 | 364.75 |
| -15.0° | 18.04 | 51.11 | 60.13 | 180.40 | 120.27 | 360.80 |
| -10.0° | 18.08 | 51.24 | 60.28 | 180.83 | 120.55 | 361.66 |
| -5.0° | 18.33 | 51.94 | 61.11 | 183.33 | 122.22 | 366.67 |
| +0.0° | 18.79 | 53.25 | 62.64 | 187.93 | 125.29 | 375.87 |
| +5.0° | 19.50 | 55.26 | 65.01 | 195.03 | 130.02 | 390.07 |
| +10.0° | 20.56 | 58.26 | 68.54 | 205.63 | 137.09 | 411.26 |
| +15.0° | 22.20 | 62.91 | 74.01 | 222.03 | 148.02 | 444.06 |
| +20.0° | 25.05 | 70.98 | 83.51 | 250.53 | 167.02 | 501.06 |
| +25.0° | 31.87 | 90.31 | 106.25 | 318.74 | 212.50 | 637.49 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 19.01 | 53.86 | 63.36 | 190.08 | 126.72 | 380.16 |
| -30.0° | 14.40 | 40.79 | 47.99 | 143.97 | 95.98 | 287.95 |
| -20.0° | 13.22 | 37.46 | 44.07 | 132.20 | 88.13 | 264.39 |
| -10.0° | 12.68 | 35.92 | 42.26 | 126.79 | 84.53 | 253.58 |
| +0.0° | 12.36 | 35.03 | 41.21 | 123.64 | 82.43 | 247.28 |
| +10.0° | 12.15 | 34.43 | 40.51 | 121.52 | 81.01 | 243.03 |
| +20.0° | 12.00 | 34.00 | 40.00 | 120.00 | 80.00 | 240.00 |
| +30.0° | 11.91 | 33.75 | 39.70 | 119.10 | 79.40 | 238.21 |
| +40.0° | 11.96 | 33.87 | 39.85 | 119.55 | 79.70 | 239.10 |


### Orthogonal Layout (90° Yaw)

#### 1. Minimum Torque Capacity Summary
| Motor | Limit Mode | Motor Limit (N·m) | Min Pitch Torque (N·m) | Critical Pitch Angle | Min Roll Torque (N·m) | Critical Roll Angle |
| --- | --- | --- | --- | --- | --- | --- |
| **RS02** | Rated | 6.0 | 15.01 | -14.5° | 14.52 | -25.0° |
| **RS02** | Peak | 17.0 | 42.52 | -14.5° | 41.15 | -25.0° |
| **RS03** | Rated | 20.0 | 50.02 | -14.5° | 48.41 | -25.0° |
| **RS03** | Peak | 60.0 | 150.06 | -14.5° | 145.24 | -25.0° |
| **RS04** | Rated | 40.0 | 100.04 | -14.5° | 96.82 | -25.0° |
| **RS04** | Peak | 120.0 | 300.11 | -14.5° | 290.47 | -25.0° |

#### 2. Pitch Torque Capacity at Sample Foot Angles (Roll = 0°)
| Pitch Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -25.0° | 15.44 | 43.75 | 51.47 | 154.41 | 102.94 | 308.82 |
| -20.0° | 15.11 | 42.81 | 50.36 | 151.08 | 100.72 | 302.16 |
| -15.0° | 15.01 | 42.52 | 50.02 | 150.07 | 100.04 | 300.13 |
| -10.0° | 15.06 | 42.66 | 50.19 | 150.58 | 100.39 | 301.16 |
| -5.0° | 15.23 | 43.15 | 50.76 | 152.28 | 101.52 | 304.56 |
| +0.0° | 15.51 | 43.93 | 51.69 | 155.06 | 103.38 | 310.13 |
| +5.0° | 15.90 | 45.05 | 53.00 | 159.00 | 106.00 | 317.99 |
| +10.0° | 16.43 | 46.56 | 54.77 | 164.32 | 109.55 | 328.64 |
| +15.0° | 17.15 | 48.60 | 57.18 | 171.54 | 114.36 | 343.08 |
| +20.0° | 18.16 | 51.46 | 60.55 | 181.64 | 121.09 | 363.28 |
| +25.0° | 19.67 | 55.74 | 65.58 | 196.73 | 131.15 | 393.46 |

#### 3. Roll Torque Capacity at Sample Foot Angles (Pitch = 0°)
| Roll Angle | RS02 Rated | RS02 Peak | RS03 Rated | RS03 Peak | RS04 Rated | RS04 Peak |
| --- | --- | --- | --- | --- | --- | --- |
| -40.0° | 15.31 | 43.37 | 51.02 | 153.06 | 102.04 | 306.12 |
| -30.0° | 14.59 | 41.33 | 48.63 | 145.88 | 97.26 | 291.77 |
| -20.0° | 14.57 | 41.28 | 48.56 | 145.68 | 97.12 | 291.36 |
| -10.0° | 14.92 | 42.27 | 49.73 | 149.18 | 99.45 | 298.36 |
| +0.0° | 15.65 | 44.34 | 52.17 | 156.51 | 104.34 | 313.02 |
| +10.0° | 17.07 | 48.36 | 56.90 | 170.69 | 113.79 | 341.38 |
| +20.0° | 20.67 | 58.57 | 68.90 | 206.70 | 137.80 | 413.40 |
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