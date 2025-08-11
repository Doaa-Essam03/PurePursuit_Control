import numpy as np
import matplotlib.pyplot as plt

# Desired start point
x_start = 0.748
y_start = 3.16

# 1. Straight Line Path
x1 = np.linspace(0, 4, 100) + x_start
y1 = np.linspace(0, 7, 100) + y_start

# 2. S-Curve Path
x2 = np.linspace(0, 9.25, 100) + x_start
y2 =  2 * np.sin(0.5 * x2) + y_start 

# 3. Circle Path (centered relative to start)
theta3 = np.linspace(0, 2 * np.pi, 100)
radius3 = 2
x3 = x_start + radius3 * np.cos(theta3)
y3 = y_start + radius3 * np.sin(theta3)

# 4. Sharp Turn Path
x4 = np.concatenate([np.linspace(0, 1.75, 50), np.full(50, 1.75)]) + x_start
y4 = np.concatenate([np.full(50, 0), np.linspace(0, 7, 50)]) + y_start

# 5. Zig-Zag Path
x5 = np.linspace(0, 9.25, 100) + x_start
y5 = y_start + 1.5 * np.sign(np.sin(2 * np.pi * x5 / 2))


# Plotting
plt.figure(figsize=(10, 10))
plt.plot(x1, y1, label="Straight Line", linestyle='--')
plt.plot(x2, y2, label="S Curve")
plt.plot(x3, y3, label="Circle")
plt.plot(x4, y4, label="Sharp Turn", linestyle=':')
plt.plot(x5, y5, label="Zig-Zag")

# Starting point marker
plt.plot(x_start, y_start, 'ro', label="Start Point")

plt.title("F1TENTH Path Shapes (All Start at Same Point)")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
