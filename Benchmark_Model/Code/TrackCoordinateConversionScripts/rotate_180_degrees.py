import matplotlib.pyplot as plt
import pandas as pd

# Load data from CSV file
# data = pd.read_csv("ReferenceLine\S10_R11&12_Shanghai_RacingLine.csv")

file_path = 'TransformedCoordinates\PortlandCoordinates.xlsx'
data = pd.read_excel(file_path)
data.head(), data.columns

x = data['x']
y = data['y']

# Rotate the coordinates 180 degrees around (0, 0)
x_rotated = -x
y_rotated = -y

rotated_data = pd.DataFrame({'x': x_rotated, 'y': y_rotated})
rotated_data.to_excel('rotated_coordinates.xlsx', index=False)

plt.figure(figsize=(10, 6))

# Plot rotated coordinates
plt.plot(x_rotated, y_rotated, marker='x', linestyle='--', color='r', label='Rotated Data points')

plt.title('Original and Rotated (180 degrees) Coordinates')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.legend()

# Show grid
plt.grid(True)

# Display the plot
plt.show()