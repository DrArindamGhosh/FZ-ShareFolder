% Load data from Excel file
filename = 'MEX_driving_line.xlsx'; % Update the path to your file
data = readtable(filename);

% Extract x, y, and orientation columns
x = data.x;
y = data.y;
orientation = data.orientation; % Notice the misspelling in your file
% sDistance = data.sDistance; 

% Convert orientation from radians to degrees (if necessary)
orientation_deg = rad2deg(orientation);

% Create a new figure
figure;

% Plot x and y using black dots
plot(x, y, 'k.');

% Hold on to add orientation vectors to the same plot
hold on;

% Length of the orientation vector
vector_length = 1; % Adjust this value as necessary

% Compute the end points of the orientation vectors
u = vector_length * cos(orientation);
v = vector_length * sin(orientation);

% Plot orientation vectors
quiver(x, y, u, v, 0, 'r');

% Set axis labels and title
xlabel('X coordinate');
ylabel('Y coordinate');
title('Orientation Vectors on X-Y Plane');

% Enable grid
grid on;

% Hold off to finish plotting
hold off;
