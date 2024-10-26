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

% 160m, 125--160m/s for 800m



x0 = [[0, 0, 230], [ 1 , 0, 0, 0], [0, 0, 125], [0, 0, 0]]; % orientation not great

%constants
data.massB = 4.2525;
data.Inertia = diag([0.035, 4.6, 4.6]);
data.Length = 1.842;
data.RailAngle = 90.0;
data.referenceArea = 81.7*10^-4;
data.controlSurfaceArea = 34.5*10^-4;
data.nomAB_DC = 1.17;
data.airBrakePosition = 1.54;
data.maxABLength = 0.021;
data.ABonDC = 0.79;
data.ABoffDC = 0.35;
data.normalDC = 13.6;
data.dampDC = 4.88;
data.stability = 0.408;
data.CoM = [0, 0, 1.05/2]; %needs to be measured for a rocket

% Dimensions of nose, body and fins (m)
data.LEN_NOSECONE = 150*10^-3;
data.ROOTCHORD_FIN = 85*10^-3;
data.TIPCHORD_FIN = 50*10^-3;
data.AVERAGECHORD_FIN = (data.ROOTCHORD_FIN + data.TIPCHORD_FIN)/2;
data.THICKNESS_FIN = 0.238*10^-3;
data.HEIGHT_FIN = 53*10^-3;
data.MIDCHORD_FIN = 53*10^-3;
data.SWEEPANGLE_FIN = 24*pi/180;
data.SWEEPLENGTH_FIN = 23.3*10^-3;
data.AREA_FIN = (data.ROOTCHORD_FIN + data.TIPCHORD_FIN)/2 * data.HEIGHT_FIN * 4;
data.DIAMETER_BODY = 60*10^-3;
data.LEN_ROCKET = 1140*10^-3;
data.KINEMATIC_VISC = 1.495*10^-5;
data.AREA_FRONTBODY = pi*data.DIAMETER_BODY^2/4;
data.maxABLength = 0.021; % == x, assumption for boundary layer
data.CD_0 = 1.17;
data.referenceArea = 81.7*10^-4;
data.controlSurfaceArea = 34.5*10^-4;

%wind = [0,0,0]; %constant for now
data.ref_roll = [0,0,1]; %roll axis

data.g = [0,0,-9.81]; %gravity
data.u = 0.0;
% var_w = 1.8*2^2*(position(3)/500)^(2/3) * (1 - 0.8 * position(3)/500)^2; %variance of wind
% std_w = sqrt(var_w);    %standard deviation
data.wind = [0,0,2];


tspan = [0 25];
h = 0.4;
%u = 0;
%Use ode15s, ode45 took too long, probably cause the system is stiff
%options = odeset("RelTol",1e-4, "AbsTol", 1e-4, "Events",@Event);
 %[t,y] = ode45(@(t,y) rocketODE(t,y), tspan, x0, options);
[t,y] = solver(@rocketODE, tspan, x0, h,data);
%% Plotting
% figure(1)
% plot3(y(:,1), y(:,2), y(:,3))
% title("Trajectory")
% %axis([0 40 0 40 0 400])
% xlabel("X disp")
% ylabel("Y disp")
% zlabel("Height")
% grid on
% 
% figure(2)
% plot(y(:,10), y(:,3),"-o")
% title("Altitude vs Speed")
% grid on