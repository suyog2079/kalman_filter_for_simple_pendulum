% Load data from file
data = load('data.txt');

% Extract columns
sensor_measurement = data(:,1);
filter_output = data(:,2);

% Create index for x-axis
time = 1:length(sensor_measurement);

% Set up the figure
figure;
xlabel('Time');
ylabel('Value');
title('Sensor Measurement vs Filter Output');
grid on;
hold on;

% Initialize animated plot
h1 = plot(NaN, NaN, 'b-', 'LineWidth', 2); % Blue line for sensor measurements
h2 = plot(NaN, NaN, 'r-', 'LineWidth', 2); % Red line for filter output

% Animate the data
for k = 1:length(time)
    set(h1, 'XData', time(1:k), 'YData', sensor_measurement(1:k));
    set(h2, 'XData', time(1:k), 'YData', filter_output(1:k));
    pause(0.05); % Adjust speed of animation
end
hold off;

