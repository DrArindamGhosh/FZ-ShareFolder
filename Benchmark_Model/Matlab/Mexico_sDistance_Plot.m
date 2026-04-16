% Load data from Excel file
filename = 'MEX_driving_line.xlsx'; % Update the path to your file
data = readtable(filename);

% Extract x, y, and sDistance columns
x = data.x;
y = data.y;
sDistance = data.sDistance;

% Create a new figure
figure;

% Scatter plot x and y, color-coded by sDistance
scatter(x, y, 20, sDistance, 'filled'); % Adjust marker size as needed

% Add color bar to indicate the scale of sDistance
colorbar;

% Set colormap (you can choose another suitable colormap)
colormap(jet);

% Set axis labels and title
xlabel('X coordinate');
ylabel('Y coordinate');
title('Scatter Plot of sDistance on X-Y Plane');

% Enable grid
grid on;
