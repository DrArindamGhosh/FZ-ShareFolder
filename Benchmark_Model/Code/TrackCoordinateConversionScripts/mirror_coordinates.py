import matplotlib.pyplot as plt
import pandas as pd

# Load data from CSV file
data = pd.read_csv('ReferenceLine\S10_R08_MCO_RacingLine.csv')

x = data['X']
y = data['Y']

# Mirror the coordinates in the line y = 0
y_mirrored = -y

mirrored_data = pd.DataFrame({'x': x, 'y': y_mirrored})
mirrored_data.to_excel('mirrored_coordinates.xlsx', index=False)

# Create a plot
plt.figure(figsize=(10, 6))


# Plot mirrored coordinates
plt.plot(x, y_mirrored, marker='x', linestyle='--', color='r', label='Mirrored Data points')

plt.title('Original and Mirrored Coordinates (in line y = 0)')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.legend()

# Show grid
plt.grid(True)

# Display the plot
plt.show()