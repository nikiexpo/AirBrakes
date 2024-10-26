clear; clc;
%% Sim settings
h = 0.4;                            %time step
tspan = [0 25];                     %time span
N = 1:20;                          %number of iterations
M = round((tspan(2) - tspan(1))/h);        %number of 
%Vdata = zeros(M,length(N));                  %velocity data
%Sdata = zeros(M,length(N));                  %height data
options = odeset("RelTol",1e-4, "AbsTol", 1e-4, "Events",@Event);
%% Monte carlo Parameters
%varying parameters
degs = unifrnd(89,91,size(N));      %pitch
p = (degs-90).*(pi/180);            %pitch in radians, centered around zero
r = zeros(size(N));                 %roll
y = zeros(size(N));                 %yaw
q = angle2quat(y,p,r);              %quaternion - yaw,pitch,roll order

s = unifrnd(160,230,size(N));       %height at burnout
v = unifrnd(120,160,size(N));       %velocity at burnout

%construct the intial condition vector
z = zeros(size(N))';                                    %zero vector
x0 = [z,z,s',q(:,1),q(:,2),q(:,3),q(:,4),z,z,v',z,z,z]; %intial conds

%% Monte Carlo

for k = 1:length(N)                                     %loop till max iter
    [t,y] = ode45(@rocketODE, tspan, x0(k,:)', options);        %solve
    %Vdata(:,k) = y(:,10);                               %store velocity
    %Sdata(:,k) = y(:,3);                                %store height
    c(:,k) = {t, y(:,10), y(:,3)};
end

%% Plots
% copySdata = Sdata;
% copyVdata = Vdata;                          
% %https://uk.mathworks.com/matlabcentral/answers/393053-how-to-omit-zero-values-while-plotting-the-graph
% copySdata(copySdata==0) = nan;
% %copyVdata(copyVdata==0) = nan;

figure(1)
for i = 1:length(N)
    ts = c{1,i};
    alt = c{3,i};
    plot(ts,alt)
    hold on
end
hold off
title("Altitude plot for different initial conditions")
grid on
xlabel("Time (s)")
ylabel("Altitude (m)")

figure(2)
for i = 1:length(N)
    ts = c{1,i};
    vel = c{2,i};
    plot(ts,vel)
    hold on
end
hold off
title("Velocity plot for different initial conditions")
grid on
xlabel("Time (s)")
ylabel("Velocity (m/s)")