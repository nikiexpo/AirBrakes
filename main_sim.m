%% Main file to run 
clear; clc; 


%% Code from here

% x = rocketDynamics;
% disp(x.massAtBurnout)
% disp(x.state')
% 
% n = 1000;
% dt = 0.1;
% Vel = zeros(n,3);
% 
% for i = 1:10
%     x = x.update(dt);
%     Vel(i,:) = x.linearVelocity';
% end

x0 = [[0, 0, 232], [1, 0, 0, 0], [0, 0, 166], [0.3, 0, 0.2]];

tspan = [0 5];
%Use ode15s, ode45 took too long, probably cause the system is stiff
[t,y] = ode15s(@rocketODE, tspan, x0);