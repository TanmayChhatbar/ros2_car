classdef Vehicle2DCSVWriter
    properties
        fid
    end

    methods
        function obj = Vehicle2DCSVWriter(output_file)
            fid = fopen(output_file, 'w');
            if fid == -1
                fprintf("Failed to open file: %s\n", output_file);
                return;
            end
            obj.fid = fid;
            obj.writeCSVHeader();
        end

        function writeCSVHeader(obj)
            header = ['time,X,Y,yaw,vx,vy,w_yaw,ax,ay,a_yaw,' ...
                'w_wheel_fl,w_wheel_fr,w_wheel_rl,w_wheel_rr,' ...
                'Fx_wheel_fl,Fx_wheel_fr,Fx_wheel_rl,Fx_wheel_rr,' ...
                'Fy_wheel_fl,Fy_wheel_fr,Fy_wheel_rl,Fy_wheel_rr,' ...
                'steering,motor_torque'];
            fprintf(obj.fid, '%s\n', header);
        end

        function WriteCSVData(o, time, obj)
            steering_angle = obj.data.steering_angle;
            motor_torque = obj.data.motor_torque;

            X = obj.data.X;
            Y = obj.data.Y;
            yaw = obj.data.yaw;
            vx = obj.data.vx;
            vy = obj.data.vy;
            w_yaw = obj.data.w_yaw;
            ax = obj.data.ax;
            ay = obj.data.ay;
            a_yaw = obj.data.a_yaw;
            w_wheel = obj.data.w_wheel;
            Fx_wheel = obj.data.Fx_wheel;
            Fy_wheel = obj.data.Fy_wheel;

            fprintf(o.fid, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n', ...
                time, ...
                X, Y, yaw, ...
                vx, vy, w_yaw, ...
                ax, ay, a_yaw, ...
                w_wheel(1), w_wheel(2), w_wheel(3), w_wheel(4), ...
                Fx_wheel(1), Fx_wheel(2), Fx_wheel(3), Fx_wheel(4), ...
                Fy_wheel(1), Fy_wheel(2), Fy_wheel(3), Fy_wheel(4), ...
                steering_angle, motor_torque);
        end

        function close(obj)
            obj.fid.close()
        end
    end
end