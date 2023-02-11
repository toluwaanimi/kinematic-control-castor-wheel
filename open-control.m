% Define the time steps and duration of the simulation
dt = 0.1;
t_max = 20;
t = 0:dt:t_max;

% Define the initial position and velocity of the robot
x0 = [0; 0; 2; pi/4];

% Define the two goals for the robot to reach
goal1 = [2; 2];
goal2 = [-2; -2];

% Define the control inputs (i.e., the wheel velocities)
u = [0.5; -0.5];

% "Define the ODE function to simulate the robot's motion"
% x(1) = x position
% x(2) = y position
% x(3) = linear velocity
% x(4) = orientation
ode_fun = @(t, x) [x(3)*cos(x(4)); x(3)*sin(x(4)); (u(1) + u(2))/2; (u(2) - u(1))/2];

% Use the ODE45 function to solve the ODE and obtain the position and orientation of the robot over time for the first goal
[t, X] = ode45(ode_fun, t, x0);

% Re-define the initial position and velocity for the second goal
x0 = X(end,:);
x0(3) = 2;

% Use the ODE45 function to solve the ODE and obtain the position and orientation of the robot over time for the second goal
[t, X2] = ode45(ode_fun, t, x0);

% Combine the position and orientation arrays for both goals
X = [X; X2];

% Plot the position of the robot over time for both goals
figure
plot(X(:,1), X(:,2), 'o-')
xlabel('X (m)')
ylabel('Y (m)')

title('Position of the Caster Wheel Robot Over Time for Both Goals')
hold on
plot(goal1(1), goal1(2), 'ro', 'MarkerSize', 10)
plot(goal2(1), goal2(2), 'ro', 'MarkerSize', 10)
legend('Robot Trajectory', 'Goal 1', 'Goal 2')



