import numpy as np
import matplotlib.pyplot as plt

# The number of points per side to pin
POINTS_PER_SIDE = 25

# The number of samples per width
SAMPLES_WIDTH = 50

# The number of samples per height
SAMPLES_HEIGHT = 50

# Build the linspaces
linspace_width = np.linspace(0, SAMPLES_WIDTH - 1, POINTS_PER_SIDE)
linspace_height = np.linspace(0, SAMPLES_HEIGHT - 1, POINTS_PER_SIDE)

points = []

# Build the top side and bottom side
for width in linspace_width:
    points.append([int(width), 0])
    points.append([int(width), SAMPLES_HEIGHT - 1])

for height in linspace_height:
    points.append([0, int(height)])
    points.append([SAMPLES_WIDTH - 1, int(height)])

print(points)
