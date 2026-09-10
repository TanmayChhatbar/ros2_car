classdef Vehicle2DData
    properties
        X
        Y
        yaw
        vx
        vy
        w_yaw
        ax
        ay
        a_yaw

        p_wheel
        w_wheel
        a_wheel

        Fx
        Fy
        Mz

        Fx_wheel
        Fy_wheel
        Fz_wheel

        steering_angle
        motor_torque
        brake_torque
        wheel_torques
    end

    methods
        function obj = Vehicle2DData()
            obj.X = 0;
            obj.Y = 0;
            obj.yaw = 0;
            obj.vx = 0;
            obj.vy = 0;
            obj.w_yaw = 0;
            obj.ax = 0;
            obj.ay = 0;
            obj.a_yaw = 0;
            obj.p_wheel = zeros(1, 4);
            obj.w_wheel = zeros(1, 4);
            obj.a_wheel = zeros(1, 4);
            obj.Fx = 0;
            obj.Fy = 0;
            obj.Mz = 0;
            obj.Fx_wheel = zeros(1, 4);
            obj.Fy_wheel = zeros(1, 4);
            obj.Fz_wheel = zeros(1, 4);
            obj.steering_angle = 0;
            obj.motor_torque = 0;
            obj.brake_torque = 0;
            obj.wheel_torques = zeros(1, 4);
        end
    end
end