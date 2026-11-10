
import numpy as np
import matplotlib.pyplot as plt 

# -----------------------------------------
# 1. Define the simplified leg
# -----------------------------------------

L1 = 0.30  # Upper leg length, meters
L2 = 0.30  # Lower leg length, meters

duration = 1.0
samples = 101

t = np.linspace(0, duration, samples)
u = t / duration

# -----------------------------------------
# 2. Define the desired foot path
# -----------------------------------------

x = -0.15 + 0.30 * u
y = -0.50 + 0.12 * np.sin(np.pi * u)

# -----------------------------------------
# 3. Calculate inverse kinematics
# -----------------------------------------

hip_angles = []
knee_angles = []

for i in range(samples):

    # Calculate the cosine of the relative knee angle
    cos_knee = (
        x[i]**2 + y[i]**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Protect against small floating-point errors
    cos_knee = np.clip(cos_knee, -1.0, 1.0)

    # Choose one knee configuration
    knee = np.arccos(cos_knee)

    # Calculate the hip angle
    hip = (
        np.arctan2(y[i], x[i])
        - np.arctan2(
            L2 * np.sin(knee),
            L1 + L2 * np.cos(knee)
        )
    )

    # Convert radians to degrees
    hip_angles.append(np.degrees(hip))
    knee_angles.append(np.degrees(knee))

hip_angles = np.array(hip_angles)
knee_angles = np.array(knee_angles)

# -----------------------------------------
# 4. Plot the foot path
# -----------------------------------------

plt.figure(figsize=(7, 6))

plt.plot(x, y, label="Desired foot path")
plt.scatter(
    [x[0], x[-1]],
    [y[0], y[-1]],
    label="Start and end",
    zorder=3
)

plt.scatter([0], [0], label="Hip", zorder=3)

# Draw the leg at several sample times
for i in [0, 25, 50, 75, 100]:

    hx = 0.0
    hy = 0.0

    knee_x = L1 * np.cos(hip_angles[i] * np.pi / 180)
    knee_y = L1 * np.sin(hip_angles[i] * np.pi / 180)

    foot_x = x[i]
    foot_y = y[i]

    plt.plot(
        [hx, knee_x, foot_x],
        [hy, knee_y, foot_y],
        marker="o",
        alpha=0.6
    )

plt.xlabel("Horizontal position x (m)")
plt.ylabel("Vertical position y (m)")
plt.title("LLEAP: 2D Foot Path and Leg Configurations")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

# -----------------------------------------
# 5. Plot joint angles over time
# -----------------------------------------

fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

axes[0].plot(t, hip_angles)
axes[0].set_ylabel("Hip angle (degrees)")
axes[0].grid(True)

axes[1].plot(t, knee_angles)
axes[1].set_ylabel("Relative knee angle (degrees)")
axes[1].set_xlabel("Time (seconds)")
axes[1].grid(True)

plt.tight_layout()
plt.show()

# -----------------------------------------
# 6. Print sample results
# -----------------------------------------

print("Time | Foot X | Foot Y | Hip angle | Knee angle")
print(" (s) |   (m)  |   (m)  |   (deg)   |   (deg)")

for i in range(0, samples, 10):
    print(
        f"{t[i]:4.2f} | "
        f"{x[i]:6.3f} | "
        f"{y[i]:6.3f} | "
        f"{hip_angles[i]:9.2f} | "
        f"{knee_angles[i]:10.2f}"
    )
