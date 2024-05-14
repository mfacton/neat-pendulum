# neat-pendulum
Double Pendulum Balancing with NEAT

### Time stepping
gravity at 0.25 gives good time step movement 

### Actions
none, left, right

### constant variables
Gravity (g)
Constant length (r1 & r2)
End Mass (m1 & m2)

### Arm 1 & 2 changing variables
Base angle (theta1 & theta2)
Angle velocity (vtheta1 & vtheta2)
Angle velocity (atheta & atheta2)

### Solution Algorithms
Euler by position, velocity, and acceleration (BAD)
Euler by energy (ok)
Runge-Kutta (good)
[More Info](https://www.myphysicslab.com/pendulum/double-pendulum-en.html)