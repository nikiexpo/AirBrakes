# AirBrakes
Simulation "working", not great, oscillations in the 3D plot, XY directions are not resolved correctly. 

MATLAB simulation is dodgy:
    1. Either fix the bugs with the current solver
    2. Or try implementing the solver

RocketPy:
    1. No support for airbrakes as of now
    2. So probably need to edit source for this (difficult)
    3. Need to find a trajectory optimisation tool box for this in python



Things to note (needs changing/wrong /correct  etc):
1. Not updating the roll axis (check Page 13 of the 6DoF paper) -- DONE but still same - but the variations are very small in X and Y dir, investigate
2. Include variations in atmos properties (not a priority)
3. gravity vector assumed was correct (check eq 27 from section 3)
4. Still no clue on how to calculate the damping coefficient
5. yet to implement calculations for some of the drag coeff
6. Quaternion derivative is updated differently compared to a normal dydt = f(t,y) -- figure out how to deal with this
7. setting angular velocity to zero caused problems previously -- check if it is the same with the new integration method
