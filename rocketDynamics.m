% high fidelity model of a rocket from *insert source* 
classdef rocketDynamics
    properties (Access = public) %change it to priv later
        massAtBurnout double
        diagonalInertia single
        rocketLength double
        launchRailAngle single
        referenceArea double
        controlSurfaceArea double
        nomAirBrakeDragCoeff single
        airBrakePosition single
        maxAirBrakeLength double
        % All in SI units
    end
    methods
        function obj = rocketDynamics()
            if nargin == 0
                obj.massAtBurnout = 4.2525;
                obj.diagonalInertia = 4.6;
                obj.rocketLength = 1.842;
                obj.launchRailAngle = 90.0;
                obj.referenceArea = 81.7*10^-4;
                obj.controlSurfaceArea = 34.5*10^-4;
                obj.nomAirBrakeDragCoeff = 1.17;
                obj.airBrakePosition = 1.54;
                obj.maxAirBrakeLength = 0.021;
            end
        end
    end
end 