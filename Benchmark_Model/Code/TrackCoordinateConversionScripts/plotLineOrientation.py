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
    # Convert orientations to range [0, 2π]
    orientations = (orientations + 2 * np.pi) % (2 * np.pi)
    # Ensure orientation continuity
    for i in range(1, len(orientations)):
        while orientations[i] - orientations[i - 1] > np.pi:
            orientations[i] -= 2 * np.pi
        while orientations[i] - orientations[i - 1] < -np.pi:
            orientations[i] += 2 * np.pi
    
    return orientations

file_path = 'coordinates_with_distance_orientation.xlsx'
data = pd.read_excel(file_path)
data.head(), data.columns

# Load data from CSV file
# data = pd.read_csv("ReferenceLine\S9_R13-14_Rome_RacingLine.csv")

x = data['x2']
y = data['y2']

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
