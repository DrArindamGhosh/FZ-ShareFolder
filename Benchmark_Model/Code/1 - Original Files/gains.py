import numpy as np

# Current gains and limits
current_gains = np.array([1, 0.5, 0.3, 0.2, 0.15, 0.1, 0.05, 0.02])
current_upper_limits = np.array([1.5, 1.5, 2, 2.75, 3, 2.75, 2.25, 1.7])
current_lower_limits = np.array([-1.5, -1.5, -2, -2.75, -3, -2.75, -2.25, -1.7])

# Calculate new gains and limits
new_gains = current_gains.copy()
new_gains[:8] *= .5  # Increasing gains for the first three elements

new_upper_limits = current_upper_limits * 0.5  # Reducing upper limits by 20%
new_lower_limits = current_lower_limits * 0.5  # Reducing lower limits by 20%

(new_gains, new_upper_limits, new_lower_limits)

print(new_gains, new_upper_limits, new_lower_limits)