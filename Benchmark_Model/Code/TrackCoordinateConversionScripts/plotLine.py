import matplotlib.pyplot as plt
import pandas as pd

# data = pd.read_csv("ReferenceLine\S10_R11&12_Shanghai_RacingLine.csv")

file_path = 'ProcessedDiLTracks\Shanghai_Traj_Mat_v3.xlsx'
data = pd.read_excel(file_path)
data.head(), data.columns

x = data['x']
y = data['y']

# x = data['x2']
# y = data['y2']

# Create plot
plt.figure(figsize=(10, 6))
plt.plot(x, y, marker='o', linestyle='-')

plt.title('Plot of X and Y Coordinates')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.legend()

# Show grid
plt.grid(True)

# Display the plot
plt.show()