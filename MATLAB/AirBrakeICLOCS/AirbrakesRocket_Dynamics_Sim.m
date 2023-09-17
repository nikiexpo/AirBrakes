function [dxdt] = AirbrakesRocket_Dynamics_Sim(x,u,p,t,vdat)
   position = [x(:,1), x(:,2), x(:,3)];
    quaternion = [x(:,4), x(:,5), x(:,6), x(:,7)];
    quaternion = quaternion ./ norm(quaternion); %normalize
    Lvelocity = [x(:,8), x(:,9), x(:,10)];
    Avelocity = [x(:,11), x(:,12), x(:,13)];

    dxdt = zeros(size(x)); %allocate memory
    [Nrows,Ncols] = size(x);
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
    dxdt(:,1) = x(:,8);   % pos dot is equal to velocity
    dxdt(:,2) = x(:,9);
    dxdt(:,3) = x(:,10);


    %quaternion dot
    q_w = quaternion(:,1);
    q_v = quaternion(:,2:end);
    q_dot = [0.5.*dot(Avelocity, q_v,2), 0.5.*(q_w.*Avelocity + cross(Avelocity, q_v,2))];
    dxdt(:,4) = q_dot(:,1);
    dxdt(:,5) = q_dot(:,2);
    dxdt(:,6) = q_dot(:,3);
    dxdt(:,7) = q_dot(:,4);

    %v dot
    Fg = massB.*g;

    R = quat2rotm(real(quaternion)); % Rotation matrix
    e_roll = zeros(Nrows,3);
    for i = 1:Nrows
        e_roll(i,:) = (R(:,:,i)*ref_roll')'; % compute the roll axis in current orientation
    end
    e_roll = e_roll ./ norm(e_roll);
    
    
    var_w = 1.8.*2^2.*(position(:,3)./500).^(2/3) .* (1 - 0.8 .* position(:,3)./500).^2; %variance of wind
    std_w = sqrt(var_w);    %standard deviation
    randome = normrnd(zeros(size(var_w)),std_w);
    %disp(randome)
    wind = [zeros(size(var_w)), zeros(size(var_w)), randome]; % zero mean normal distribution of wind
    [~, a, ~, rho] = atmoscoesa(position(:,3));

    [CN,CoP] = CoeffCalculator();
    stab = zeros(Nrows,3);
    for i = 1:Nrows
        stab(i,:)  = CoP - CoM;
    end
    V_cop = Lvelocity + cross(Avelocity, stab ,2);
    V_app = V_cop + wind;
    n_Vapp = zeros(Nrows,1) ;
    for i = 1:Nrows
        n_Vapp(i,1) = norm(V_app(i,:));
    end
    %u = u; %needs a function so evaluate u
    [CD] = dragCoeffCalculator(n_Vapp,a,u,vdat);

    Fa = ((-rho/2).*CD.*referenceArea.* n_Vapp.^2).*e_roll;
    AoA = acos(dot((V_app./n_Vapp), e_roll,2)); %angle of attack, radians 
    CD_n = CN .* AoA; 

    Fn = ((rho/2).*CD_n.*referenceArea.* n_Vapp.^2).*(cross(e_roll, cross(e_roll, (V_app./n_Vapp),2),2));
    vdot = (1/massB).*(Fn + Fa + Fg);

    dxdt(:,8) = vdot(:,1);
    dxdt(:,9) = vdot(:,2);
    dxdt(:,10) = vdot(:,3);
    %disp(Lvelocity(3));
    %w dot
    n_Fn = zeros(Nrows,1);
    for i = 1:Nrows
        n_Fn(i,1) = norm(Fn(i,:));
    end
    stability = norm(CoP - CoM);
    torque_n = (stability .* n_Fn) .* cross(e_roll, (V_app./n_Vapp),2);
    torque_damp = zeros(size(torque_n));
    wdot = zeros(Nrows,3);
    for i = 1:Nrows
        torque_damp(i,:) = ((-1*dampDC).*(R(:,:,i)*diag([1 1 0])/R(:,:,i))*Avelocity(i,:)')';
        wdot(i,:) = Inertia\(torque_damp(i,:)+torque_n(i,:))'; 
    end
    
    dxdt(:,11) = wdot(:,1);
    dxdt(:,12) = wdot(:,2);
    dxdt(:,13) = wdot(:,3);
end