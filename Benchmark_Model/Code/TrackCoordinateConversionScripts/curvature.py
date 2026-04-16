import pandas as pd
import numpy as np

def calculate_radius_of_curvature(df, x_col='x', y_col='y', s_col='sDistance'):
    """
    Calculate the radius of curvature for each point in the trajectory.

    Parameters:
    df (DataFrame): Input DataFrame containing 'x', 'y', and 'sDistance' columns.
    x_col (str): Column name for x-coordinate.
    y_col (str): Column name for y-coordinate.
    s_col (str): Column name for the distance along the trajectory.

    Returns:
    DataFrame: DataFrame with an additional 'Radius_of_Curvature' column.
    """
    # Extract relevant columns
    x = df[x_col].values
    y = df[y_col].values
    s = df[s_col].values

    # Compute first derivatives (velocities) using central differences
    dx_ds = np.gradient(x, s)
    dy_ds = np.gradient(y, s)

    # Compute second derivatives (accelerations) using central differences
    d2x_ds2 = np.gradient(dx_ds, s)
    d2y_ds2 = np.gradient(dy_ds, s)

    # Calculate curvature
    curvature = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / (dx_ds**2 + dy_ds**2)**(3/2)

    # Add the radius of curvature to the dataframe
    df['Curvature'] = curvature

    return df


file_path = 'trajectory_with_curvature.xlsx'
df = pd.read_excel(file_path)

df_with_radius = calculate_radius_of_curvature(df)

df_with_radius.to_excel('trajectory_with_curvature.xlsx', index=False)

print("Radius of curvature calculation completed and saved.")
