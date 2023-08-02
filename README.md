# AirBrakes
## Simulation 
Rocket model (ODE) for the simulator is ready, can be found in the MATLAB folder

1. The coefficient of damping still needs work but the rest are reasonably accurate. 
2. Needs more testing 

The control algorithm needs to be added.

SO : Idea #1 : 
1. Create a trajectory with boundary condition apogee 800m
2. trajectory is based on vertical speed and height 
3. LQR to follow this trajectory




RocketPy:
    1. No support for airbrakes as of now
    2. So probably need to edit source for this (difficult)
    3. Need to find a trajectory optimisation tool box for this in python



