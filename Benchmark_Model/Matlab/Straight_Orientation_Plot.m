% Load data from Excel file
filename = 'straight_line.xlsx'; % Update the path to your file
data = readtable(filename);

% Extract x, y, and orientation columns
x = data.x;
y = data.y;
orientation = data.orientation; % Correct for any misspellings as in your previous file

% Create a new figure
figure;

% Plot x and y using a line
plot(x, y, 'b-o'); % 'b-o' specifies blue color and circle markers
hold on;

% Plot orientation vectors
vector_length = 1; % Define the length of the orientation vectors
u = vector_length * cos(orientation);
v = vector_length * sin(orientation);

% Add orientation vectors using quiver
quiver(x, y, u, v, 0, 'r'); % '0' auto-scales vectors, 'r' specifies red color

% Set axis labels and title
xlabel('X coordinate');
ylabel('Y coordinate');
title('Orientation and Trajectory on X-Y Plane');

% Enable grid
grid on;

% Finish plotting
hold off;
