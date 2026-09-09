#!/usr/bin/env python3

import pandas as pd
import numpy as np

"""
===============================================================================
Module: find_start_index.py
Description:
    Finds the closest trajectory waypoint index to a given initial vehicle pose.
===============================================================================
"""

# Load CSV
df = pd.read_csv('/home/doaa/f1_tenth/src/pure_pursuit_pkg/pure_pursuit_pkg/csv_paths_practice_cdc/Centerline_less_points.csv')

# Coordinates of starting point
start_x, start_y = 0.8, 3.16
print(df.columns)

# Compute Euclidean distance to all points
distances = np.sqrt((df['positions_X'] - start_x)**2 + (df['positions_y'] - start_y)**2)

# Get the index of the closest point
index = distances.idxmin()

print(f"Closest point is at index: {index}, coordinates: ({df['positions_X'][index]}, {df['positions_y'][index]})")
print(distances.min())
