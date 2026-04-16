import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def calculate_cumulative_distance(x, y):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_distance = np.concatenate(([0], np.cumsum(distances)))
    return cumulative_distance

def calculate_orientation(x, y):
    orientations = np.arctan2(np.diff(y), np.diff(x))
    orientations = np.concatenate(([orientations[0]], orientations))
    
    # Ensure orientation continuity within [-π, π]
    for i in range(1, len(orientations)):
        while orientations[i] - orientations[i - 1] > np.pi:
            orientations[i] -= 2 * np.pi
        while orientations[i] - orientations[i - 1] < -np.pi:
            orientations[i] += 2 * np.pi

    # Normalize orientations to range [0, π] for left and right hemispheres
    orientations = (orientations + np.pi) % (2 * np.pi) - np.pi
    orientations = np.abs(orientations)  # Take absolute value to map to [0, π]

    return orientations

file_path = 'ProcessedReferenceLines\Portland_Traj_Mat.xlsx'
data = pd.read_excel(file_path)

x = data['x']
y = data['y']

# Calculate cumulative distance and orientation
cumulative_distance = calculate_cumulative_distance(x, y)
orientation = calculate_orientation(x, y)

# Add new columns to the DataFrame
data['sDistance'] = cumulative_distance
data['Orientation'] = orientation

data.to_excel('coordinates_with_distance_orientation.xlsx', index=False)

# Create a scatter plot with color representing orientation
plt.figure(figsize=(10, 6))
sc = plt.scatter(x, y, c=orientation, cmap='hsv', marker='o', label='Data points')

# Add a color bar
cbar = plt.colorbar(sc)
cbar.set_label('Orientation (radians)')

plt.title('Coordinates with Orientation Color Plot')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.legend()

# Show grid
plt.grid(True)

# Display the plot
plt.show()

# Print the new DataFrame
print(data)