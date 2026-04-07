classdef PIDControl
    properties
        kp
        ki
        kd
        dt
        windup_limit
        integral
        prev_error
    end
    methods
        function obj = PIDControl(kp, ki, kd, dt, windup_limit)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.dt = dt;
            obj.windup_limit = windup_limit;
            obj.integral = 0;
            obj.prev_error = 0;
        end

        function c = compute(obj, setpoint, measured_value)
            error = setpoint - measured_value;

            obj.integral = obj.integral + error * obj.dt;
            obj.integral = min(max(obj.integral, -obj.windup_limit), obj.windup_limit);

            derivative = (error - obj.prev_error) / obj.dt;
            obj.prev_error = error;

            c = obj.kp * error + obj.ki * obj.integral + obj.kd * derivative;
        end

        function obj = reset(obj)
            obj.prev_error = 0;
            obj.integral = 0;
        end

        function obj = setWindupLimit(obj, limit)
            obj.windup_limit = limit;
        end
    end
end