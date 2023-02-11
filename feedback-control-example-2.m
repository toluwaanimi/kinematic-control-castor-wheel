% Define the desired velocities and control gains
desired_velocities = [0.9, 0.5]; % The first value represents the desired linear velocity and the second value represents the desired angular velocity
control_gains = [0., 0.1]; % The first value represents the linear velocity control gain and the second value represents the angular velocity control gain

% Set the initial conditions for the system
x0 = 0; % Initial x-position
y0 = 0; % Initial y-position
theta0 = 0; % Initial orientation

% Define the time range for each goal
time_range1 = [0, 2]; % Time range for the first goal
time_range2 = [3, 6]; % Time range for the second goal

options = optimoptions('fmincon','Algorithm','trust-region-reflective');

% Solve the ODE for each goal using ode45
% The function "feedback_control_function" will be called to compute the derivative of the state vector for each time step
% The initial state [x0, y0, theta0] is passed as the last argument to ode45
[time1, state1] = ode45(@(t, X) feedback_control_function(t, X, desired_velocities(1), desired_velocities(2), control_gains), time_range1, [x0, y0, theta0],options);
[time2, state2] = ode45(@(t, X) feedback_control_function(t, X, desired_velocities(2), desired_velocities(1), control_gains), time_range2, state1(end,:),options);

% Combine the solutions from both goals
% The time vectors are concatenated and the state matrices are concatenated, starting from the second row of time2 and state2
time = [time1; time2(2:end)];
state = [state1; state2(2:end,:)];

% Plot the results
plot(state(:,1), state(:,2)); % Plot the x-position versus the y-position
xlabel('x'); % Label the x-axis
ylabel('y'); % Label the y-axis

% Define the ODE function
function X_dot = feedback_control_function(~, X, vd, wd, kv_kw)
    % Extract the state variables from the state vector
    x = X(1);
    y = X(2);
    theta = X(3);
    % Extract the control gains from the kv_kw vector
    kv = kv_kw(1);
    kw = kv_kw(2);

    % Compute the error between the desired and current state
    ex = vd * cos(theta) - x;
    ey = vd * sin(theta) - y;
    etheta = wd - theta;

    % Compute the control inputs using feedback control
    v = vd + kv * ex;
    w = wd + kw * etheta;

    % Compute the derivative of the state
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = w;

    % Return the derivative of the state as a column vector
    X_dot = [x_dot; y_dot; theta_dot];
end