function [dxdt] = AirbrakesRocket_Dynamics_Sim(x,u,p,t,vdat)
    position = [x(1), x(2), x(3)];
    quaternion = [x(4), x(5), x(6), x(7)];
    quaternion = quaternion ./ norm(quaternion); %normalize
    Lvelocity = [x(8), x(9), x(10)];
    Avelocity = [x(11), x(12), x(13)];

    dxdt = zeros(13,1); %allocate memory

    %constants
    massB = 4.2525;
    Inertia = diag([0.035, 4.6, 4.6]);
    %Length = 1.842;
    %RailAngle = 90.0;
    referenceArea = 81.7*10^-4;
    %controlSurfaceArea = 34.5*10^-4;
    %nomAB_DC = 1.17;
    %airBrakePosition = 1.54;
    %maxABLength = 0.021;
    %ABonDC = 0.79;
    %ABoffDC = 0.35;
    %normalDC = 13.6;
    dampDC = 4.88;
    %stability = 0.408;
    CoM = [0, 0, 1.05/2]; %needs to be measured for a rocket

    %wind = [0,0,0]; %constant for now
    ref_roll = [0,0,1]; %roll axis
    
    g = [0,0,-9.81]; %gravity

    %postion dot
    dxdt(1) = x(8);   % pos dot is equal to velocity
    dxdt(2) = x(9);
    dxdt(3) = x(10);


    %quaternion dot
    q_w = quaternion(1);
    q_v = quaternion(2:end);
    q_dot = [0.5.*dot(Avelocity, q_v), 0.5.*(q_w.*Avelocity + cross(Avelocity, q_v))];
    dxdt(4) = q_dot(1);
    dxdt(5) = q_dot(2);
    dxdt(6) = q_dot(3);
    dxdt(7) = q_dot(4);

    %v dot
    Fg = massB.*g;

    R = quat2rotm(real(quaternion)); % Rotation matrix
    e_roll = (R*ref_roll')'; % compute the roll axis in current orientation
    e_roll = e_roll ./ norm(e_roll);
    
    
    var_w = 1.8*2^2*(position(3)/500)^(2/3) * (1 - 0.8 * position(3)/500)^2; %variance of wind
    std_w = sqrt(var_w);    %standard deviation
    randome = normrnd(0,std_w);
    %disp(randome)
    wind = [0, 0, randome]; % zero mean normal distribution of wind
    [~, a, ~, rho] = atmoscoesa(position(3));

    [CN,CoP] = CoeffCalculator();
    V_cop = Lvelocity + cross(Avelocity, (CoP - CoM));
    V_app = V_cop + wind;
    n_Vapp = V_app ./ norm(V_app);
    
    %u = u; %needs a function so evaluate u
    [CD] = dragCoeffCalculator(norm(V_app),a,u);

    Fa = ((-rho/2)*CD*referenceArea* norm(V_app)^2).*e_roll;
    AoA = acos(dot(n_Vapp, e_roll)); %angle of attack, radians 
    CD_n = CN * AoA; 

    Fn = ((rho/2)*CD_n*referenceArea* norm(V_app)^2).*(cross(e_roll, cross(e_roll, n_Vapp)));
    vdot = (1/massB).*(Fn + Fa + Fg);

    dxdt(8) = vdot(1);
    dxdt(9) = vdot(2);
    dxdt(10) = vdot(3);
    %disp(Lvelocity(3));
    %w dot
    
    stability = norm(CoP - CoM);
    torque_n = (stability * norm(Fn)) .* cross(e_roll, n_Vapp);
    torque_damp = ((-1*dampDC).*(R*diag([1 1 0])/R)*Avelocity')';
    wdot = Inertia\(torque_damp+torque_n)'; 

    dxdt(11) = wdot(1);
    dxdt(12) = wdot(2);
    dxdt(13) = wdot(3);
end