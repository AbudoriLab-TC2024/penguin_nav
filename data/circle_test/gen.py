import math

r = 5.0
interval = 1.0
num_points = int(2 * math.pi * r / interval)

points = []
for i in range(num_points):
    x = r * math.cos(i * interval / r - math.pi / 2)
    y = r * math.sin(i * interval / r - math.pi / 2) + 5
    print(f"{x:.3f},{y:.3f}")

print("0.000,0.000")
