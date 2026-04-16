import math
import csv
import pandas as pd

# Load the CSV file
df = pd.read_csv('01mexico_gps_coordinates.csv')

# Convert the values in columns 'C' and 'D' to float
df['Longitude1'] = pd.to_numeric(df['Longitude1'], errors='coerce')
df['Latitude1'] = pd.to_numeric(df['Latitude1'], errors='coerce')

# Save the DataFrame back to CSV
df.to_csv('01mexico_gps_coordinates.csv', index=False)

earth_radius = 6371

# def convert_to_cartesian(latitude, longitude):
#     # Convert latitude and longitude to radians
#     lat_rad = math.radians(latitude)
#     lon_rad = math.radians(longitude)


#     # Calculate Cartesian coordinates
#     x = earth_radius * math.cos(lat_rad) * math.cos(lon_rad)
#     y = earth_radius * math.cos(lat_rad) * math.sin(lon_rad)
#     z = earth_radius * math.sin(lat_rad)

#     return x, y, z

# # Example usage
# latitude = 37.7749
# longitude = -122.4194
# cartesian_coords = convert_to_cartesian(latitude, longitude)




def convert_csv_to_cartesian(input_file, output_file):
    # Read input CSV file
    with open(input_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header row
        coordinates = []
        for row in reader:
            try:
                if row[3] == '' or row[2] == '':
                    break
                coordinates.append((float(row[3]), float(row[2])))
            except ValueError:
                print(f"Error converting to float in row: {row}")
                continue

        # Convert coordinates to Cartesian
        cartesian_coords = []
        for lat, lon in coordinates:
            lat_rad = math.radians(lat)
            lon_rad = math.radians(lon)
            x = earth_radius * math.cos(lat_rad) * math.cos(lon_rad)
            y = earth_radius * math.cos(lat_rad) * math.sin(lon_rad)
            z = earth_radius * math.sin(lat_rad)
            cartesian_coords.append((x, y, z))  # Append the coordinates to the list

        if cartesian_coords:  # Check if the list is not empty
            first_point = cartesian_coords[0]
            transformed_coords = [(x - first_point[0], y - first_point[1], z - first_point[2]) for x, y, z in cartesian_coords]
            cartesian_coords = transformed_coords
    

    # Write output CSV file
    with open(output_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X', 'Y', 'Z'])  # Write header row
        writer.writerows(cartesian_coords)

# Example usage
input_file = '01mexico_gps_coordinates.csv'
output_file = 'mexico_convert.csv'
convert_csv_to_cartesian(input_file, output_file)




import matplotlib.pyplot as plt

# Read the output CSV file
df_output = pd.read_csv(output_file)

# Plot the X and Y coordinates
plt.scatter(df_output['X'], df_output['Y'])
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Plot of X and Y Coordinates')
plt.show()