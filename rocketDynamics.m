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
        dragCoeffRocket single
        controlInput single = 0;
    end
    properties (Access = public) %change it to private
        % All vectors are column vectors as default
        position (3,1) double  %x,y,z
        quaternion (4,1) double  %w,i,j,k convention
        linearVelocity (3, 1) double  %Vx, Vy, Vz
        angularVelocity (3,1) double  %Wx, Wy, Wz
        state (13, 1) double % the full state with all the above vectors
    end
    methods
        function obj = rocketDynamics()
            if nargin == 0
                %initialize the rocket parameters
                obj.massAtBurnout = 4.2525;
                obj.diagonalInertia = 4.6;
                obj.rocketLength = 1.842;
                obj.launchRailAngle = 90.0;
                obj.referenceArea = 81.7*10^-4;
                obj.controlSurfaceArea = 34.5*10^-4;
                obj.nomAirBrakeDragCoeff = 1.17;
                obj.airBrakePosition = 1.54;
                obj.maxAirBrakeLength = 0.021;
                obj.dragCoeffRocket = 0.2;
                
                %initialize the state
                obj.position = [0, 0, 0]';
                obj.quaternion = [1, 0, 0, 0]';
                obj.linearVelocity = [0, 0, 0]';
                obj.angularVelocity = [0, 0, 0]';
                obj.state = [obj.position', obj.quaternion', obj.linearVelocity', obj.angularVelocity']';
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
            g = -9.81;
            F_g = obj.massAtBurnout*g;
            
            [~, a, ~, rho] = atmosisa(obj.position(3)); %density and speed of sound from isa model
            C_A_rocket = obj.dragCoeffRocket / sqrt(1 - (obj.linearVelocity(end)/a)^2 ); %compressibility compensated axial drag coeff

            delta = 0.7; % BOUNDARY LAYER THICKNESS :: NEEDS TO BE CALCULATED; PLACEHOLDER FOR NOW  
            C_A_brakes = obj.nomAirBrakeDragCoeff * (1 - 0.25*(delta/obj.maxAirBrakeLength));      %axial drag coef on airbrakes

            C_A_total = C_A_rocket + obj.controlInput * (obj.controlSurfaceArea / obj.referenceArea) * C_A_brakes; % total axial drag coeff
            F_a = (-1*rho/2) * C_A_total; %continue later
        end
    end
end 