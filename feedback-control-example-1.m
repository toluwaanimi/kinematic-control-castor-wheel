% Desired velocities

vd1 = 0.9;
wd1 = 0.5;
vd2 = 0.5;
wd2 = 0.2;

% Control gains
kv = 0.5;
kw = 0.1;
% Initial conditions
x0 = 0;
y0 = 0;
theta0 = 0;

% Time span
tspan1 = [0, 2];
tspan2 = [3, 6];

% Solve the ODE using ode45 for first goal

options = optimoptions('fmincon','Algorithm','trust-region-reflective');

[t1, X1] = ode45(@(t,X) feedback_control_function(t, X, vd1, wd1, kv, kw), tspan1, [x0, y0, theta0], options);

% Get the final state after first goal

x_final = X1(end,1);
y_final = X1(end,2);

theta_final = X1(end,3);

% Solve the ODE using ode45 for second goal

[t2, X2] = ode45(@(t,X) feedback_control_function(t, X, vd2, wd2, kv, kw), tspan2, [x_final, y_final, theta_final], options);

% Combine the solutions from both goals
t = [t1; t2(2:end)];
X = [X1; X2(2:end,:)];

% Plot the results
plot(X(:,1), X(:,2))
xlabel('x')
ylabel('y')

% ODE function

function X_dot = feedback_control_function(~, X, vd, wd, kv, kw)
    x = X(1);
    y = X(2);
    theta = X(3);
    % Compute the error
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
    X_dot = [x_dot; y_dot; theta_dot];
end

