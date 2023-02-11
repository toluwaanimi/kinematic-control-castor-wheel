
% Define the initial conditions and time step

x0 = [2; 5]; % initial angle and angular velocity (rad, rad/s)
tspan = [0, 10]; % time span for the simulation (s)

% Define the two goals
theta_goal_1 = pi/2; % first goal angle (rad)
theta_goal_2 = -pi/2; % second goal angle (rad)

% Compute the LQR control input
% State weight matrix
Q = diag([1, 1]);
% Control weight matrix
R = 1;
% LQR gain matrix
[K,S,e] = lqr(A,B,Q,R);
% Control input function
u = @(t) -K*(x0-[theta_goal_1; 0]);

% Run the ODE simulation
[t, x] = ode45(@(t,x) caster_wheel_lqr_ode(t,x,u), tspan, x0);

% Plot the results
figure;
subplot(2,1,1);
plot(t, x(:,1));
hold on;
plot([0, max(t)], [theta_goal_1, theta_goal_1], '--');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(2,1,2);
plot(t, x(:,2));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');



% Define the ODE function
function xdot = caster_wheel_lqr_ode(t,x,u)
    % Parameters
    m = 1; % mass of the robot (kg)
    l = 0.5; % length of the robot (m)
    r = 0.1; % radius of the caster wheel (m)
    g = 9.8; % acceleration due to gravity (m/s^2)

    % States
    theta = x(1); % angle of the robot (rad)
    omega = x(2); % angular velocity of the robot (rad/s)

    % Control input
    u = u(t); % control input at time t (Nm)

    % Equations of motion
    thetadot = omega;
    omegadot = -g/l*sin(theta) + u/m/l^2;

    % Return the derivative of the states
    xdot = [thetadot; omegadot];
end


