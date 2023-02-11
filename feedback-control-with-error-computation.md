## Feedback Control with Error Computation


This code uses feedback control to enable the robot to travel between two goals. The ODE45 function is used to solve the differential equations and the control inputs are adjusted based on the current position of the robot relative to the goals. The control gains (K) are used to regulate the magnitude of the error and hence the control inputs (v and omega). The plot shows the robot's trajectory as it travels between the two goals.