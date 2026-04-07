classdef VehicleCruiseControl
    properties
        target_speed
        kp
        ki
        kd
        dt
        throttle_input
        brake_input
        prev_error
        integral
    end

    methods
        function obj = VehicleCruiseControl(target_speed_, kp_, ki_, kd_, dt_)
            obj.target_speed = target_speed_;
            obj.kp = kp_;
            obj.ki = ki_;
            obj.kd = kd_;
            obj.dt = dt_;
            obj.throttle_input = 0;
            obj.brake_input = 0;
            obj.prev_error = 0;
            obj.integral = 0;
        end

        function [obj, throttle_input, brake_input] = calcCruiseControlInputs(obj, vx)
            error = obj.target_speed - vx;
            obj.integral = obj.integral + error * obj.dt;
            
            derivative = (error - obj.prev_error) / obj.dt;
            throttle_input = obj.kp * error + obj.ki * obj.integral + obj.kd * derivative;
            throttle_input = max(min(1, throttle_input), -1);
            brake_input = 0;
            obj.prev_error = error;
        end
    end
end
