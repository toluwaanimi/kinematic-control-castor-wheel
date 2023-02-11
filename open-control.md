
## Open Control Example


The code provided is a solution to the motion control problem of a Caster Wheel robot by using open loop control with a constant input. The robot needs to get from its initial positionx0 to two different goals (goal1 and goal2). The two-dimensional coordinates (x,y) of these goals are defined at the beginning. The control inputs u[0.5; -0.5] (wheel velocities of both wheels) set the speed and direction of the robot. After definition of t (time steps) and dt (duration of the simulation) an ODE function is defined that takes the initial position x0 of the robot as well as the control inputs u into account. Two calls of the ODE45 solver are used - one for each goal - where the initial conditions for the second call are set to the end values of the first call. Finally, the trajectory of the robot over time is plotted in a graph, showing the start and goals.