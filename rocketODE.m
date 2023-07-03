function dxdt = rocketODE(t, x)
    position = [x(1), x(2), x(3)];
    quaternion = [x(4), x(5), x(6), x(7)];
    Lvelocity = [x(8), x(9), x(10)];
    Avelocity = [x(11), x(12), x(13)];
    dxdt = zeros(13,1);
    %constants
    massB = 4.2525;
    Inertia = diag([0.035, 4.6, 4.6]);
    Length = 1.842;
    RailAngle = 90.0;
    referenceArea = 81.7*10^-4;
    controlSurfaceArea = 34.5*10^-4;
    nomAB_DC = 1.17;
    airBrakePosition = 1.54;
    maxABLength = 0.021;
    ABonDC = 0.79;
    ABoffDC = 0.35;
    normalDC = 13.6;
    dampDC = 4.88;
    %stability = 0.408;

    wind = [0,0,10]; %constant for now
    e_roll = [0,0,1]; %roll axis
    CoP = [0,0,0.2]; %dist from CoM, arbritrary for now
    g = [0,0,-9.81]; %gravity, check sign and compute actual vector later

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

    V_cop = position + cross(Avelocity, (CoP - position));
    V_app = V_cop + wind;
    n_Vapp = V_app ./ norm(V_app);
    [~, a, ~, rho] = atmoscoesa(position(3));

    CD_r = 0.2502* 1/sqrt(1 - norm(Lvelocity)/a);
    CD_a = CD_r; %no control input for now

    Fa = ((-rho/2)*CD_a*referenceArea* norm(V_app)^2).*e_roll;
    AoA = acos(dot(n_Vapp, e_roll)); %angle of attack, radians for now
    CD_n = normalDC * AoA; %asuming normalDC is constant for now, it may not be

    Fn = ((-rho/2)*CD_n*referenceArea* norm(V_app)^2).*(cross(e_roll, cross(e_roll, V_app)));
    vdot = (1/massB).*(Fn + Fa + Fg);

    dxdt(8) = vdot(1);
    dxdt(9) = vdot(2);
    dxdt(10) = vdot(3);

    %w dot
    R = quat2rotm(quaternion);
    stability = norm(CoP - position);
    torque_n = (stability * norm(Fn)) .* cross(e_roll, V_app);
    torque_damp = (-1*dampDC).*(R*diag([1 1 0])/R)*Lvelocity';
    wdot = Inertia\(torque_damp+torque_n); 

    dxdt(11) = wdot(1);
    dxdt(12) = wdot(2);
    dxdt(13) = wdot(3);
end