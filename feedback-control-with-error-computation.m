% Define the control gains
K = [0.1, 0.1, 0.1];

% Define the initial conditions
x0 = [0; 0; 0];

% Define the first goal
goal1 = [1; 2; pi/2];

% Define the second goal
goal2 = [-1; 3; -pi/2];

% Define the time interval
tspan = [0, 10];

% Define the options for ODE45
options = odeset('RelTol', 1e-4, 'AbsTol', [1e-4, 1e-4, 1e-4]);

% Solve the ODE using the ODE45 function
[t, x] = ode45(@(t, x) feedback_control_law(t, x, K, goal1, goal2), tspan, x0, options);

% Plot the trajectory of the robot
plot(x(:,1), x(:,2), 'b', 'LineWidth', 2);
hold on;

% Plot the first goal
plot(goal1(1), goal1(2), 'ro', 'LineWidth', 2);

% Plot the second goal
plot(goal2(1), goal2(2), 'ro', 'LineWidth', 2);

% Add labels and title to the plot
xlabel('x');
ylabel('y');
title('Robot Trajectory using Feedback Control');



function dx = feedback_control_law(t, x, K, goal1, goal2)

% Define the state variables
x_r = x(1);
y_r = x(2);
theta_r = x(3);

% Check if the robot is closer to the first goal or the second goal
if (norm([x_r-goal1(1); y_r-goal1(2)]) < norm([x_r-goal2(1); y_r-goal2(2)]))
    goal = goal1;
else
    goal = goal2;
end

% Compute the error between the current state and the goal
error = [goal(1)-x_r; goal(2)-y_r; goal(3)-theta_r];

% Compute the control inputs using the control gains
v = K(1)*error(1);
omega = K(2)*error(2) + K(3)*error(3);

% Define the state derivatives
dx = [v*cos(theta_r); v*sin(theta_r); omega];
end

