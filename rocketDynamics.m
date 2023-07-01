% high fidelity model of a rocket from *insert source* 
classdef rocketDynamics
    properties (Access = public) %change it to priv later
        % All in SI units
        massAtBurnout double
        diagonalInertia single
        rocketLength double
        launchRailAngle single
        referenceArea double
        controlSurfaceArea double
        nomAirBrakeDragCoeff single
        airBrakePosition single
        maxAirBrakeLength double 
        stabilityMargin double
        
        controlInput single = 0;
        positionOfCoP (3,1) double
        %Environment
        windVeloctiy (3,1) double
        %coeff of drag
        dragCoeffABon single
        dragCoeffABoff single
        normalDragCoeff single
        dampDragCoeff single
    end
    properties (Access = public) %change it to private
        % All vectors are column vectors as default
        position (3,1) double  %x,y,z
        quaternion (4,1) double  %w,i,j,k convention
        linearVelocity (3, 1) double  %Vx, Vy, Vz
        angularVelocity (3,1) double  %Wx, Wy, Wz
        state (13, 1) double % the full state with all the above vectors
        normalForce (3,1) double
        rollAxis (3,1) single
        apparentVelocity (3,1) double
    end
    methods
        function obj = rocketDynamics()
            if nargin == 0
                %initialize the rocket parameters
                obj.massAtBurnout = 4.2525;
                obj.diagonalInertia = diag([0.035, 4.6, 4.6]);
                obj.rocketLength = 1.842;
                obj.launchRailAngle = 90.0;
                obj.referenceArea = 81.7*10^-4;
                obj.controlSurfaceArea = 34.5*10^-4;
                obj.nomAirBrakeDragCoeff = 1.17;
                obj.airBrakePosition = 1.54;
                obj.maxAirBrakeLength = 0.021;
                obj.dragCoeffABon = 0.35;
                obj.dragCoeffABoff = 0.79;
                obj.normalDragCoeff = 13.6;
                obj.dampDragCoeff = 4.88;
                obj.stabilityMargin = 0.408;

                %initlaize environment
                obj.windVeloctiy = [10, 0 , 0]'; %constant wind for now (its a function of alt) -- see eq 27

                %initialize the state
                obj.position = [0, 0, 0]';
                obj.quaternion = [1, 0, 0, 0]';
                obj.linearVelocity = [0, 0, 0]';
                obj.angularVelocity = [0, 0, 0]';
                obj.state = [obj.position', obj.quaternion', obj.linearVelocity', obj.angularVelocity']';
                obj.positionOfCoP = [0, 0, 0.2]'; %depends on OR sim or maybe find a way to calculate CoP
                obj.rollAxis = [0 0 1]; % assuming this for now, not sure, double check
            end
        end
        
%         function [new_state] = update(dt)
% 
%         end
% 
%         function [r_k] = positionUpdate(obj,dt)
%             r_k = obj.position + obj.velocity.*dt;
%         end
%         function [q_k] = quaternionUpdate(obj,dt)
%             q_k(1,1) =  obj.quaternion(1,1) + 0.5 * dot(obj.angularVelocity, obj.quaternion(2:end, 1)) * dt;
%             q_k(2:end, 1) = obj.quaternion(2:end, 1) + 0.5 .* (obj.quaternion(1,1).*obj.angularVelocity + cross(obj.angularVelocity',obj.quaternion(2:end, 1)')').*dt;
%         end

        % time derivative of each state 
        function [v] = r_dot(obj)
            v = obj.linearVelocity;
        end
        function [q] = q_dot(obj)
            q_w = obj.quaternion(1);
            q_v = obj.quaternion(2:end);
            omega = obj.angularVelocity;
            q = [0.5.*(dot(omega, q_v)), 0.5.*(q_w.*omega + cross(omega, q_v))];
        end
        function [v] = v_dot(obj)
            g = [0 0 -9.81]'; % wrong, gravity vector changes over time https://forum.arduino.cc/t/3d-maths-to-sutract-gravity-from-accelerometer-data/258105/10
            F_g = obj.massAtBurnout.*g; % this also should be a vector, need to compute the gravity vector
            
            [~, a, ~, rho] = atmosisa(obj.position(3)); %density and speed of sound from isa model
            C_A_rocket = obj.dragCoeffABon / sqrt(1 - (obj.linearVelocity(end)/a)^2 ); %compressibility compensated axial drag coeff
            
            % CALCULATION AVOIDED FOR NOW
            %delta = 0.7; % BOUNDARY LAYER THICKNESS :: NEEDS TO BE CALCULATED; PLACEHOLDER FOR NOW  
            %C_A_brakes = obj.nomAirBrakeDragCoeff * (1 - 0.25*(delta/obj.maxAirBrakeLength));      %axial drag coef on airbrakes

            C_A_brakes = obj.dragCoeffABon - obj.dragCoeffABoff;
            C_A_total = C_A_rocket + obj.controlInput * (obj.controlSurfaceArea / obj.referenceArea) * C_A_brakes; % total axial drag coeff

            e_roll = obj.rollAxis; % assuming this for now, not sure, double check

            V_CoP = obj.linearVelocity + cross(obj.angularVelocity, (obj.positionOfCoP - obj.position))';
            V_app = V_CoP + obj.windVeloctiy;
            obj.apparentVelocity = V_app;
            norm_V_app = V_app / norm(V_app);
            F_A = ((-1*rho/2) * C_A_total * obj.referenceArea * norm(V_app)^2).*e_roll; % this should be a vector but dont know how to compute e_roll yet eq 24b
            
            AoA = acosd(dot(norm_V_app, e_roll));
            C_N = obj.normalDragCoeff*AoA;
            F_N = ((-1*rho/2) * C_N * obj.referenceArea * norm(V_app)^2).*(cross(e_roll, cross(e_roll, norm_V_app)));
            obj.normalForce = F_N;
            v = (1/obj.massAtBurnout).*(F_g + F_N + F_A);
        end
        function omega = omega_dot(obj)
            torque_N = (obj.stabilityMargin * norm(obj.normalForce)).*cross(obj.rollAxis, obj.apparentVelocity);
    
            %Constructing the rotation matrix 
            R = quat2rotm(obj.quaternion);

            damping_torque = (-obj.dampDragCoeff).*((R*diag([1 1 0])/R)*obj.angularVelocity);
            omega = I\(torque_N + damping_torque);
        end
    end
end 