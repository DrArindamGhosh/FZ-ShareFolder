# import numpy as np

# def adjust_yaw_angles(yaw_angles):
#     """Ensure yaw angles are continuous and adjusted without normalization."""
#     if not yaw_angles:
#         return []

#     for i in range(1, len(yaw_angles)):
#         # Ensure continuity based on the next angle generated which is between 0 and π
#         while yaw_angles[i] - yaw_angles[i - 1] > np.pi:
#             yaw_angles[i] -= np.pi
#         while yaw_angles[i] - yaw_angles[i - 1] < -np.pi:
#             yaw_angles[i] += np.pi

#     return yaw_angles

# # Example usage
# yaw_angles = [0.5, 1.0, 1.5, 2.0, 3.0, 0.5, 1.0, 1.5, 2.0, 1.0, 0.1, ]
# adjusted_yaw_angles = adjust_yaw_angles(yaw_angles)
# print(adjusted_yaw_angles)

# import numpy as np

# def adjust_yaw_angles(yaw_angles):
#     """Ensure yaw angles are continuous and adjusted without normalization."""
#     if not yaw_angles:
#         return []

#     adjusted_yaw_angles = [yaw_angles[0]]
    
#     for i in range(1, len(yaw_angles)):
#         new_angle = yaw_angles[i]
        
#         # Check if the new angle needs to be adjusted
#         while new_angle - adjusted_yaw_angles[-1] > 1:
#             new_angle -= 2 * np.pi
#         while new_angle - adjusted_yaw_angles[-1] < 1:
#             new_angle += 2 * np.pi

#         adjusted_yaw_angles.append(new_angle)

#     return adjusted_yaw_angles

# # Example usage
# yaw_angles = [0.5, 1.0, 1.5, 2.0, 2.5, 0.1, 1.5, 2.0, 2.5]
# adjusted_yaw_angles = adjust_yaw_angles(yaw_angles)
# print(adjusted_yaw_angles)

# import numpy as np

# def ensure_continuous_yaw(yaw_angles):
#     """Ensure yaw angles are continuous."""
#     if not yaw_angles:
#         return []

#     adjusted_yaw_angles = [yaw_angles[0]]
    
#     for i in range(1, len(yaw_angles)):
#         current_angle = yaw_angles[i]
#         previous_angle = adjusted_yaw_angles[-1]
        
#         # Adjust the current angle to ensure continuity
#         if current_angle < previous_angle and (previous_angle - current_angle) > np.pi:
#             current_angle += 2 * np.pi
#         elif current_angle > previous_angle and (current_angle - previous_angle) > np.pi:
#             current_angle -= 2 * np.pi
        
#         adjusted_yaw_angles.append(current_angle)

#     return adjusted_yaw_angles

# # Example usage
# yaw_angles = [3.14, 0, 0.1, 0.2, 0.3, 0.2, 0.1, 3.14, 3.13]
# adjusted_yaw_angles = ensure_continuous_yaw(yaw_angles)
# print(adjusted_yaw_angles)

# import numpy as np

# def adjust_numbers(numbers):
#     adjusted_numbers = [numbers[0]]  # Initialize the array with the first value
    
#     for i in range(1, len(numbers)):
#         last_number = adjusted_numbers[-1]
#         new_number = numbers[i]
        
#         if abs(new_number - last_number) > 1:
#             # If the difference is greater than 1, adjust the new number
#             if new_number > last_number:
#                 new_number = np.pi + new_number
#             else:
#                 new_number = np.pi - new_number
        
#         # Add the new number to the array
#         adjusted_numbers.append(new_number)
    
#     return adjusted_numbers

# # Example usage:
# n = [3.14, 0, 0.1, 0.2, 0.3, 0.2, 0.1, 3.14, 3.13]
# adjusted_numbers = adjust_numbers(n)
# print(adjusted_numbers)

# import numpy as np

# def adjust_numbers(numbers):
#     adjusted_numbers = [numbers[0]]  # Initialize the array with the first value
    
#     for i in range(1, len(numbers)):
#         last_number = adjusted_numbers[-1]
#         new_number = numbers[i]
        
#         if abs(new_number - last_number) > 1:
#             # Adjust the new number based on the rules provided
#             new_number = np.pi + new_number
        
#         # Add the new number to the array
#         adjusted_numbers.append(new_number)
    
#     return adjusted_numbers

# # Example usage:
# n = [3.14, 0, 0.1, 0.2, 0.3, 0.2, 0.1, 3.14, 3.13]
# # n = [0.3, 0.2, 0.1, 0, 3.14, 3.13, 3.12, 3.13, 3.14, 0, 0.1]
# adjusted_numbers = adjust_numbers(n)
# print(adjusted_numbers)

import numpy as np

def adjust_numbers(numbers):
    adjusted_numbers = [numbers[0]]  # Initialize the array with the first value
    
    for i in range(1, len(numbers)):
        last_number = adjusted_numbers[-1]
        new_number = numbers[i]
        print(f"Last number: {last_number}, New number: {new_number}")
        
        if new_number - last_number < 1:
            # If the difference is greater than 1, add π to the new number
            new_number = np.pi + new_number
            print(f"{new_number} = π + {new_number}")
        
        if last_number - new_number < -1:
            # If the difference is less than -1, subtract the new number from π
            new_number = (np.pi - new_number) * -1
            print(f"{new_number} = {np.pi} - {new_number}")
        
        # Add the new number to the array
        adjusted_numbers.append(new_number)
    
    return adjusted_numbers

# Example usage:
# n = [3.14, 0, 0.1, 0.2, 0.3, 0.2, 0.1, 3.14, 3.13]
# n = [0.3, 0.2, 0.1, 0, 3.14, 3.13, 3.12, 3.13, 3.14, 0, 0.1]

# n = [0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6000000000000001, 0.7000000000000001, 0.8, 0.9, 1.0, 1.1, 1.2000000000000002, 1.3, 1.4000000000000001, 1.5, 1.6, 1.7000000000000002, 1.8, 1.9000000000000001, 2.0, 2.1, 2.2, 2.3000000000000003, 2.4000000000000004, 2.5, 2.6, 2.7, 2.8000000000000003, 2.9000000000000004, 3.0, 3.1,
#      0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6000000000000001, 0.7000000000000001, 0.8, 0.9, 1.0, 1.1, 1.2000000000000002, 1.3, 1.4000000000000001, 1.5, 1.6, 1.7000000000000002, 1.8, 1.9000000000000001, 2.0, 2.1, 2.2, 2.3000000000000003, 2.4000000000000004, 2.5, 2.6, 2.7, 2.8000000000000003, 2.9000000000000004, 3.0, 3.1]
# n = n[::-1]
# print(n)
n = [2.5, 2.6, 2.7, 2.8000000000000003, 2.9000000000000004, 3.0, 3.1, 0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.4, 0.3, 0.2, 0.1, 0, 3.14, 3.141, 0.1234, 3.1111, 0.002, 0.001, 3.123]
adjusted_numbers = adjust_numbers(n)
print(adjusted_numbers)