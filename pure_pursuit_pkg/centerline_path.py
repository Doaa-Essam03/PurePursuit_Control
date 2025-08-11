#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
path_df = pd.read_csv('/home/doaa/f1_tenth/src/pure_pursuit_pkg/pure_pursuit_pkg/csv_paths_practice_cdc/Centerline_points.csv')  

# Extract x and y columns
x = path_df['positions_X']
y = path_df['positions_y']

# Plot the path
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='.', markersize=.5 , linestyle='None', color='m', label='Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path from CSV')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
