# AirBrakes

Trajectroy Optimisation for precise apogee control using variable drag system i.e. AirBrakes. 

## Optimal Control Problem

The problem was formulated as an optimal control problem with 6dof nonlinear model of an amatuer rocket. The dynamics had 13 states including 3x1 vector of position, linear and angular velocity and 4x1 quaternion vector. 
The model itself was taken from the appendix of this [paper](https://www.eucass.eu/doi/EUCASS2019-0388.pdf) by Thomas Lew et al.

## Running the code

The problem is solved using [ICLOCS2](http://www.ee.ic.ac.uk/ICLOCS/) and note that the code also uses functions from UAV toolbox (although this can be avoided by writing a few extra lines of code). 
Airbrake2 and Airbrake3 are two different examples with different rocket parameters and targets. 

## Future work

I no longer plan to pursue building a rocket to test this however the plan was to run this as a monte-carlo simulation and then store the solution as lookup table which will be used by the microcontroller online. 


