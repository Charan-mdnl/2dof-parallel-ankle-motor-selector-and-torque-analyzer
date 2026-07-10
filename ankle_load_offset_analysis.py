import numpy as np
import matplotlib.pyplot as plt

# Parameters
M = 42.0  # mass in kg
g = 9.81  # gravity m/s^2
W = M * g  # Weight force in N
foot_length = 0.15  # 15 cm
z_uj = 0.036  # 3.6 cm ankle height
pitch_range = np.arange(-25, 26, 1)  # -25 to +25 degrees
offsets = [-0.04, -0.02, 0.0, 0.02, 0.04]  # ankle offsets in m (-4cm, -2cm, 0, 2cm, 4cm)

# Ground contact points (heel = -foot_length/2, toe = +foot_length/2)
x_heel = -foot_length / 2.0
x_toe = foot_length / 2.0

plt.figure(figsize=(10, 6))

print("==========================================================================")
print("              ANKLE PITCH TORQUE REQUIREMENT FOR 42 KG LOAD               ")
print("==========================================================================")
print(f"Vertical Load: {M} kg ({W:.2f} N)")
print(f"Foot Length: {foot_length*100:.1f} cm | Ankle Height: {z_uj*100:.1f} cm")
print("Offsets represent ankle joint position relative to the foot center (0 cm).")
print("Negative offset = closer to heel, Positive offset = closer to toe.")
print("Torque values are in N·m (absolute values represent joint effort).")
print("==========================================================================\n")

# Print table header
header = "Pitch Angle | " + " | ".join([f"Offset {off*100:+.0f}cm" for off in offsets])
print(header)
print("-" * len(header))

# Sweep and calculate
for p_deg in [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]:
    p_rad = np.radians(p_deg)
    row_vals = []
    for off in offsets:
        # Determine active contact point based on pitch direction
        if p_deg > 0:
            x_contact = x_heel
        elif p_deg < 0:
            x_contact = x_toe
        else:
            x_contact = 0.0 # neutral
            
        # Torque to support the load:
        # tau = W * (z_uj * sin(p) - (x_contact - off) * cos(p))
        tau = W * (z_uj * np.sin(p_rad) - (x_contact - off) * np.cos(p_rad))
        row_vals.append(f"{abs(tau):6.1f} N·m")
    print(f"    {p_deg:+3d}°   | " + " | ".join(row_vals))

# Calculate curves for the plot
for off in offsets:
    torques = []
    for p_deg in pitch_range:
        p_rad = np.radians(p_deg)
        if p_deg > 0:
            x_contact = x_heel
        elif p_deg < 0:
            x_contact = x_toe
        else:
            x_contact = 0.0
        tau = W * (z_uj * np.sin(p_rad) - (x_contact - off) * np.cos(p_rad))
        torques.append(abs(tau))
    plt.plot(pitch_range, torques, label=f"Ankle Offset: {off*100:+.1f} cm", linewidth=2)

plt.title(f"Ankle Pitch Torque Requirement vs Pitch Angle (42 kg Vertical Load)\nFoot Length: {foot_length*100:.1f} cm, Ankle Height: {z_uj*100:.1f} cm", fontsize=12)
plt.xlabel("Foot Pitch Angle (degrees)", fontsize=11)
plt.ylabel("Required Ankle Torque (N·m)", fontsize=11)
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend(fontsize=10)
plt.savefig("ankle_load_offset_analysis.png", dpi=150)
print("\nPlot saved successfully to 'ankle_load_offset_analysis.png'")
