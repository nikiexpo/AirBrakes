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
    end
    properties (Access = public) %change it to private
        % All vectors are column vectors as default
        position (3,1) double  %x,y,z
        quaterion (4,1) double  %w,i,j,k convention
        linearVeloctiy (3, 1) double  %Vx, Vy, Vz
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
                
                %initialize the state
                obj.position = [0, 0, 0]';
                obj.quaterion = [1, 0, 0, 0]';
                obj.linearVeloctiy = [0, 0, 0]';
                obj.angularVelocity = [0, 0, 0]';
                obj.state = [obj.position', obj.quaterion', obj.linearVeloctiy', obj.angularVelocity']';
            end
        end
    end
end 