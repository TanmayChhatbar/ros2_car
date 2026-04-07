classdef VehicleInput
    properties
        steering_input  % [rad]
        throttle_input  % [-1 1]
        brake_input     % [0 1] 
    end
    methods
        function obj = VehicleInput()
            obj.steering_input = 0;  % Initialize steering input
            obj.throttle_input = 0;  % Initialize throttle input
            obj.brake_input = 0;     % Initialize brake input
        end
    end
end