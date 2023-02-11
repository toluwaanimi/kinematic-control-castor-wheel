



This code uses motion control through feedback control to move the robot from one goal to another. The desired velocities, control gains and initial conditions are set first. Then, the ODE is solved using ode45 for both goals. The state of the robot is combined from the solutions obtained from each goal, and the results are plotted.

The control law implemented in this code computes an error between the desired velocity and the actual velocity at each time step, and adjusts the control inputs (based on the determined errors) using the predefined control gains (kv and kw). As a result, the robot can reach the target (in terms of desired state such as position and orientation) while following a smooth path in a short amount of time.

The kinematic control formula implemented here is:

xdot = [vcos(x(3)); vsin(x(3)); w];

where v is the velocity and w is the angular velocity. This formula implements the robot's motion control by taking the derivative of the robot's current state with respect to time and combining it with the control inputs.