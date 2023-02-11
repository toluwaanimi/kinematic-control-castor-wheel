% Define the initial state of the robot
x0 = [0; 0; 0];

% Define the time span for the simulation
tspan = [0, 20];

% Define the control gains for x, y, and theta
kp_x = [1, 2, 5];
kp_y = [1, 2, 5];
kp_theta = [1, 2, 5];

% Loop over the different control gain values
for i = 1:length(kp_x)
    for j = 1:length(kp_y)
        for k = 1:length(kp_theta)

            % Set the current control gains for x, y, and theta
            kp_x_current = kp_x(i);
            kp_y_current = kp_y(j);
            kp_theta_current = kp_theta(k);

            % Call the ODE solver to simulate the robot's motion
            [t, x] = ode45(@(t,x) controllaw(t, x, kp_x_current, kp_y_current, kp_theta_current), tspan, x0);

            % Plot the robot's motion
            plot_robot_motion(t, x);

            % Analyze the performance of the current control parameters
            analyze_performance(t, x, kp_x_current, kp_y_current, kp_theta_current);
        end
    end
end

% Define the control law function
function dx = controllaw(t, x, kp_x, kp_y, kp_theta)
    % Define the desired state of the robot
    xd = [5*cos(0.5*t); 5*sin(0.5*t); 0];

    % Compute the error between the current state and the desired state
    e_x = xd(1) - x(1);
    e_y = xd(2) - x(2);
    e_theta = xd(3) - x(3);

    % Compute the control inputs based on the control gains and the error
    u_x = kp_x * e_x;
    u_y = kp_y * e_y;
    u_theta = kp_theta * e_theta;

    % Compute the derivative of the state using the control inputs
    dx = [u_x * cos(x(3)); u_y * sin(x(3)); u_theta];
end

% "Define a function to plot the robot's motion"
function plot_robot_motion(t, x)
    figure;
    plot(x(:,1), x(:,2), 'LineWidth', 2);
    xlabel('x (m)');
    ylabel('y (m)');
    title('Robot Motion Trajectory');
end

% Define a function to analyze the performance of the control parameters
function analyze_performance(t, x, kp_x, kp_y, kp_theta)
    % Compute the position error
    e_x = 5*cos(0.5*t) - x(:,1);
    e_y = 5*sin(0.5*t) - x(:,2);

    % Compute the mean squared error
    mse = mean([e_x.^2; e_y.^2]);

    % Print the results
    fprintf('Control gains: kp_x = %.1f, kp_y = %.1f, kp_theta = %.1f\n', kp_x, kp_y, kp_theta);
    fprintf('Mean squared error: %.2f\n\n', mse);
end
