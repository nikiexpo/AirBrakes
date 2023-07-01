%% Main file to run 
clear; clc; 


%% Code from here

x = rocketDynamics;
disp(x.massAtBurnout)
disp(x.state')

n = 1000;
dt = 0.1;
Vel = zeros(n,3);

for i = 1:10
    x = x.update(dt);
    Vel(i,:) = x.linearVelocity';
end