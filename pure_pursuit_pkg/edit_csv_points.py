#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
path_df = pd.read_csv('/home/doaa/f1_tenth/src/pure_pursuit_pkg/pure_pursuit_pkg/csv_paths_practice_cdc/Centerline_points.csv')

# Keep every 10th row
reduced_df = path_df.iloc[::5].reset_index(drop=True)

# Save to a new CSV file
reduced_df.to_csv('/home/doaa/f1_tenth/src/pure_pursuit_pkg/pure_pursuit_pkg/csv_paths_practice_cdc/Centerline_less_points.csv', index=False)

# Optional: Plot the reduced path
x = reduced_df['positions_X']
y = reduced_df['positions_y']

plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o', linestyle='-', color='c', label='Reduced Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Reduced Path (Every 10th Point)')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
