% Load the data from the Excel file
filename = 'MEX_Steering_Inputs_Trajectory_Data.xlsx';
data = readtable(filename);

% Extract the x, y, and steering_input columns
x = data.x;
y = data.y;
steering_inputs = data.steering_input;

% Create a scatter plot of the steering inputs
figure;
scatter(x, y, 20, steering_inputs, 'filled'); % The '20' sets the marker size
colorbar; % Adds a color bar to indicate the values of steering inputs
xlabel('X Position');
ylabel('Y Position');
title('Steering Input at Different Positions');
grid on;

% Enhance visibility with more color
colormap(jet);
