function dxdt = rocketODE(t, x, data)
    position = [x(1), x(2), x(3)];
    quaternion = [x(4), x(5), x(6), x(7)];
    quaternion = quaternion ./ norm(quaternion); %normalize
    Lvelocity = [x(8), x(9), x(10)];
    Avelocity = [x(11), x(12), x(13)];

    dxdt = zeros(13,1); %allocate memory

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
    Fg = data.massB.*data.g;

    R = quat2rotm(real(quaternion)); % Rotation matrix
    e_roll = (R*data.ref_roll')'; % compute the roll axis in current orientation
    e_roll = e_roll ./ norm(e_roll);
    
    
    %wind stuff removed
    
    [~, a, ~, rho] = atmoscoesa(position(3));

    [CN,CoP] = CoeffCalculator();
    V_cop = Lvelocity + cross(Avelocity, (CoP - data.CoM));
    V_app = V_cop + data.wind;
    n_Vapp = V_app ./ norm(V_app);
    
    %cost
    %cost = controller(x(3),800);
    %disp(cost)

    %u = 0; %needs a function so evaluate u
    [CD] = dragCoeffCalculator(norm(V_app),a,data);

    Fa = ((-rho/2)*CD*data.referenceArea* norm(V_app)^2).*e_roll;
    AoA = acos(dot(n_Vapp, e_roll)); %angle of attack, radians 
    CD_n = CN * AoA; 

    Fn = ((rho/2)*CD_n*data.referenceArea* norm(V_app)^2).*(cross(e_roll, cross(e_roll, n_Vapp)));
    vdot = (1/data.massB).*(Fn + Fa + Fg);

    dxdt(8) = vdot(1);
    dxdt(9) = vdot(2);
    dxdt(10) = vdot(3);
    %disp(Lvelocity(3));
    %w dot
    
    stability = norm(CoP - data.CoM);
    torque_n = (stability * norm(Fn)) .* cross(e_roll, n_Vapp);
    torque_damp = ((-1*data.dampDC).*(R*diag([1 1 0])/R)*Avelocity')';
    wdot = data.Inertia\(torque_damp+torque_n)'; 

    dxdt(11) = wdot(1);
    dxdt(12) = wdot(2);
    dxdt(13) = wdot(3);
    
end