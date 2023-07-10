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

x0 = [[0, 0, 160], [1, 0, 0, 0], [0, 0, 203], [0, 0, 0]];

tspan = [0 25];
h = 0.4;
%Use ode15s, ode45 took too long, probably cause the system is stiff
% options = odeset("RelTol",1e-4, "AbsTol", 1e-4, "Events",@Event);
% [t,y] = ode15s(@rocketODE, tspan, x0, options);
[t,y] = solver(@rocketODE, tspan, x0, h);
%% Plotting
figure(1)
plot3(y(:,1), y(:,2), y(:,3))
title("Trajectory")
%axis([0 40 0 40 0 400])
xlabel("X disp")
ylabel("Y disp")
zlabel("Height")
grid on

figure(2)
plot(y(:,10), y(:,3),"-o")
title("Altitude vs Speed")
grid on