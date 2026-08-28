classdef Vehicle2D < matlab.mixin.Copyable
    properties
        data
        config
        input
    end

    methods
        %% constructor
        function obj = Vehicle2D(config_)
            obj.data = Vehicle2DData();
            if ~isa(config_, 'Vehicle2DConfig')
                error('Argument for Vehicle2D constructor must be a Vehicle2DConfig object')
            end
            obj.config = config_;
            obj.input = VehicleInput();
        end

        %% methods
        function obj = calcMotorTorque(obj)
            Tmax = obj.config.Tmax;
            Tnegmax = obj.config.Tnegmax;
            Tzero = obj.config.Tzero;
            motor_speed = mean(obj.data.w_wheel(3:4)) * obj.config.gear_ratio;

            % power limit
            T_Pmax = obj.config.Pmax / abs(motor_speed);
            T_Pnegmax = obj.config.Pnegmax / abs(motor_speed);

            % handle negative throttle
            if (sign(obj.input.throttle_input) == sign(motor_speed))
                motor_torque = obj.input.throttle_input * (min(Tmax, T_Pmax) + Tzero) - abs(Tzero)*sign(motor_speed);
            else
                motor_torque = obj.input.throttle_input * (min(Tnegmax, T_Pnegmax) + Tzero) - abs(Tzero)*sign(motor_speed);
            end
            obj.data.motor_torque = motor_torque;
        end

        function obj = calcBrakeTorque(obj)
            brake_bias = obj.config.brake_bias;
            brake_Tmax = obj.config.brake_Tmax;
            halfBrakeTorqueFront = obj.input.brake_input * brake_Tmax * brake_bias / 2;
            halfBrakeTorqueRear = obj.input.brake_input * brake_Tmax * (1 - brake_bias) / 2;
            w_wheel = obj.data.w_wheel;

            signs = sign(w_wheel) .* min(1, abs(w_wheel * 10));

            brakeTorque = signs .* [halfBrakeTorqueFront halfBrakeTorqueFront halfBrakeTorqueRear halfBrakeTorqueRear];
            obj.data.brake_torque = brakeTorque;
        end

        function obj = calcSteeringAngle(obj)
            obj.data.steering_angle = obj.input.steering_input * obj.config.steer_max;
        end

        function obj = calcTractionTorques(obj)
            switch obj.config.drivetrain_type
                case 2
                    obj.calcTractionTorquesFWD();
                case 1
                    obj.calcTractionTorquesAWD();
                case 0
                    obj.calcTractionTorquesRWD();
                otherwise  % "RWD" or default
                    error("Invalid drivetrain_type")
            end
        end

        function obj = calcTractionTorquesRWD(obj)
            % get data
            w_wheel = obj.data.w_wheel;
            brake_torque = obj.data.brake_torque;
            diff_damping = obj.config.diff_damping;
            motor_torque = obj.data.motor_torque;
            gear_ratio = obj.config.gear_ratio;

            % common torques
            net_wheel_torque = motor_torque * gear_ratio / 2;
            damping_torque_rear = diff_damping * diff(w_wheel(3:4));

            % wheel torques
            wheel_torques = [0 0 net_wheel_torque+[-1 1]*damping_torque_rear];
            wheel_torques = wheel_torques - brake_torque;

            obj.data.wheel_torques = wheel_torques;
        end

        function obj = calcTractionTorquesAWD(obj)
            % get data
            w_wheel = obj.data.w_wheel;
            brake_torque = obj.data.brake_torque;
            diff_damping = obj.config.diff_damping;
            motor_torque = obj.data.motor_torque;
            gear_ratio = obj.config.gear_ratio;

            % common torques
            net_wheel_torque = motor_torque * gear_ratio / 4;
            damping_torque_front = diff_damping * diff(w_wheel(1:2));
            damping_torque_rear = diff_damping * diff(w_wheel(3:4));
            damping_torque_fr = diff_damping * sum(w_wheel.*[1 1 -1 -1])/4*0; % disabled

            % wheel torques
            wheel_torques = net_wheel_torque + ...
                [-1 1 0 0]*damping_torque_front + ...
                [0 0 -1 1]*damping_torque_rear + ...
                [1 1 -1 -1]*damping_torque_fr;
            wheel_torques = wheel_torques - brake_torque;

            obj.data.wheel_torques = wheel_torques;
        end

        function obj = calcTractionTorquesFWD(obj)
            % get data
            w_wheel = obj.data.w_wheel;
            brake_torque = obj.data.brake_torque;
            diff_damping = obj.config.diff_damping;
            motor_torque = obj.data.motor_torque;
            gear_ratio = obj.config.gear_ratio;

            % common torques
            net_wheel_torque = motor_torque * gear_ratio / 2;
            damping_torque_front = diff_damping * diff(w_wheel(3:4));

            % wheel torques
            wheel_torques = [net_wheel_torque+[-1 1]*damping_torque_front 0 0];
            wheel_torques = wheel_torques - brake_torque;

            obj.data.wheel_torques = wheel_torques;
        end

        function obj = calcWheelSlipsAndForces(obj)
            % get vehicle states
            steering_angle = obj.data.steering_angle;
            a = obj.config.a;
            b = obj.config.wheelbase - a;
            half_track_width = obj.config.track_width / 2;
            r_wheel = obj.config.r_wheel;
        
            w_wheel = obj.data.w_wheel;
            Fz_wheel = obj.data.Fz_wheel;
            vx = obj.data.vx;
            vy = obj.data.vy;
            w_yaw = obj.data.w_yaw;
        
            % calculate velocities at wheel centers
            vx_yaw = half_track_width * w_yaw;
            vxw = vx + [-1 1 -1 1] * vx_yaw;
            vyw = vy + [a a -b -b] * w_yaw;

            % resolve x and y velocities in wheel frame for wheels that steer
            vxw_temp = vxw(1:2);
            vyw_temp = vyw(1:2);
            vxw(1:2) = vxw_temp * cos(steering_angle) + vyw_temp * sin(steering_angle);
            vyw(1:2) = vyw_temp * cos(steering_angle) - vxw_temp * sin(steering_angle);
        
            % calculate slip angles
            slip_angle = atan2(vyw, vxw);
            if isa(vyw, 'sym')
                slip_ratio = sym('slip_ratio', size(vxw));
            else
                slip_ratio = nan(1, 4);
            end
        
            % calculate tire forces
            slip_threshold = 0.5; % threshold for slip ratio
            for i = 1:4
                % calc slip ratios
                if isa(vyw, 'sym')
                    if i <= 2
                        % front wheels spin slower than kinematic
                        denominator = vxw(i);
                        assume(vxw(i) >= w_wheel(i) * r_wheel)
                    else
                        % rear wheels spin faster than kinematic
                        denominator = w_wheel(i) * r_wheel;
                        assume(vxw(i) <= w_wheel(i) * r_wheel)
                    end
                else
                    denominator = max(max(abs(w_wheel(i) * r_wheel), abs(vxw(i))), slip_threshold);
                end
                slip_ratio(i) = (w_wheel(i) * r_wheel - vxw(i)) / denominator;
        
                % calculate tire forces
                [Fx_wheel(i), Fy_wheel(i)] = obj.config.tire_config.calcTireForces(slip_angle(i), slip_ratio(i), w_wheel(i), Fz_wheel(i));
            end
            obj.data.Fx_wheel = Fx_wheel;
            obj.data.Fy_wheel = Fy_wheel;
        end

        function obj = calcTireNormalLoads(obj)
            % get vehicle parameters
            g = 9.81;
            m = obj.config.mass;
            a = obj.config.a;
            b = obj.config.wheelbase - a;
            h = obj.config.z_cg;
            wheelbase = obj.config.wheelbase;
            track_width = obj.config.track_width;
            ax = obj.data.ax;
            ay = obj.data.ay;
        
            Fz_part = m * g / wheelbase / 2;
            Fz_front = Fz_part * b;
            Fz_rear = Fz_part * a;
        
            % calc load transfer
            dFz_x = (h * m * ax / wheelbase) / 2;
            dFz_y = (h * m * ay / track_width) / 2;
        
            % calc and set normal loads on each wheel
            Fz_wheel = [
                Fz_front - dFz_x - dFz_y, ... % front left
                Fz_front - dFz_x + dFz_y, ... % front right
                Fz_rear + dFz_x - dFz_y,  ... % rear left
                Fz_rear + dFz_x + dFz_y   ... % rear right
            ];
        
            % check for negative normal loads
            neg_count = 0;
            Fz_neg = 0;
            if ~isa(Fz_wheel, 'sym')
                for i = 1:4
                    if (Fz_wheel(i) < 0)
                        neg_count = neg_count + 1;
                        Fz_neg = Fz_neg + Fz_wheel(i);
                        Fz_wheel(i) = 0;
                    end
                end
                % if neg count, distribute loads evenly
                if neg_count > 0
                    for i = 1:4
                        if Fz_wheel(i) > 0
                            Fz_wheel(i) = Fz_wheel(i) + Fz_neg / (4 - neg_count);
                        end
                    end
                end
            end
            obj.data.Fz_wheel = Fz_wheel;
        end

        function [F, M] = calcAerodynamicForces(obj)
            F = zeros(3,1);
            M = zeros(3,1);
            
            vx = obj.data.vx;
            % vy = obj.data.vy;
            w_yaw = obj.data.w_yaw;
            if (isa(obj.data.w_yaw, 'sym'))
                F = sym(F);
                M = sym(M);
            end
            F(1) = -0.5 * obj.config.CDx * obj.config.rho * obj.config.Af * vx^2;
            M(3) = -0.5 * obj.config.CMz * obj.config.rho * obj.config.Af * w_yaw^2;
        end

        function obj = calcNetForcesAndMoments(obj)
            steering_angle = obj.data.steering_angle;
            a = obj.config.a;
            track_width = obj.config.track_width;
            wheelbase = obj.config.wheelbase;
            b = wheelbase - a;
            half_track_width = track_width / 2;
            Fx_wheel = obj.data.Fx_wheel;
            Fy_wheel = obj.data.Fy_wheel;

            [F_aero, M_aero] = obj.calcAerodynamicForces();
        
            Fx = F_aero(1);
            Fy = F_aero(2);
            Mz = M_aero(3);
        
            % resolve forces in vehicle frame for wheels that steer
            Fx_body = zeros(1, 4);
            Fy_body = zeros(1, 4);
            if isa(Fx_wheel, 'sym')
                Fx_body = sym(Fx_body);
                Fy_body = sym(Fy_body);
            end
            for i = 1:2
                Fx_temp = Fx_wheel(i);
                Fx_body(i) = Fx_temp * cos(steering_angle) - Fy_wheel(i) * sin(steering_angle);
                Fy_body(i) = Fx_temp * sin(steering_angle) + Fy_wheel(i) * cos(steering_angle);
                Fx = Fx + Fx_body(i);
                Fy = Fy + Fy_body(i);
            end
            % resolve forces in vehicle frame for wheels that do not steer
            for i = 3:4
                Fx_body(i) = Fx_wheel(i);
                Fy_body(i) = Fy_wheel(i);
                Fx = Fx + Fx_body(i);
                Fy = Fy + Fy_body(i);
            end

            % calculate moments about the center of mass
            Mz = Mz + -Fx_body(1) * half_track_width + Fy_body(1) * a; % front left
            Mz = Mz +  Fx_body(2) * half_track_width + Fy_body(2) * a;  % front right
            Mz = Mz + -Fx_body(3) * half_track_width - Fy_body(3) * b; % rear left
            Mz = Mz +  Fx_body(4) * half_track_width - Fy_body(4) * b;  % rear right
        
            obj.data.Fx = Fx;
            obj.data.Fy = Fy;
            obj.data.Mz = Mz;
        end

        function obj = calcBodyAccelerations(obj)
            % calculate resultant body accelerations based on net forces and moments
            mass = obj.config.mass;
            Izz = obj.config.Izz;

            Fx = obj.data.Fx;
            Fy = obj.data.Fy;
            Mz = obj.data.Mz;
        
            % vx = obj.data.vx;
            % vy = obj.data.vy;
            % w_yaw = obj.data.w_yaw;
            % vehicle_slip_angle = atan2(vy, vx);
        
            obj.data.ax = Fx / mass;
            obj.data.ay = Fy / mass;
            obj.data.a_yaw = Mz / Izz;
        end

        function obj = calcWheelAccelerations(obj)
            I_wheel = obj.config.I_wheel;
            wheel_radius = obj.config.r_wheel;
        
            wheel_torques = obj.data.wheel_torques;
            Fx_wheel = obj.data.Fx_wheel;

            a_wheel = (wheel_torques - Fx_wheel.*wheel_radius) / I_wheel;
            differential_type = obj.config.differential_type;
            if (differential_type == 1)
                drivetrain_type = obj.config.drivetrain_type;
                if (drivetrain_type == 0 || drivetrain_type == 1)
                    a_wheel_rear = (a_wheel(3) + a_wheel(4)) / 2;
                    a_wheel(3) = a_wheel_rear;
                    a_wheel(4) = a_wheel_rear;
                end
                if (drivetrain_type == 2 || drivetrain_type == 1)
                    a_wheel_front = (a_wheel(1) + a_wheel(2)) / 2;
                    a_wheel(1) = a_wheel_front;
                    a_wheel(2) = a_wheel_front;
                end
            end
            obj.data.a_wheel = a_wheel;
        end

        function obj = calcNewState(obj, dt)
            ax = obj.data.ax;
            ay = obj.data.ay;
            a_yaw = obj.data.a_yaw;
        
            vx = obj.data.vx;
            vy = obj.data.vy;
            w_yaw = obj.data.w_yaw;
        
            X = obj.data.X;
            Y = obj.data.Y;
            yaw = obj.data.yaw;
        
            % calculate new velocities
            vx = vx + (ax + w_yaw * vy) * dt;
            vy = vy + (ay - w_yaw * vx) * dt;
            w_yaw = w_yaw + a_yaw * dt;
        
            % calculate new position and orientation
            sin_yaw = sin(yaw);
            cos_yaw = cos(yaw);
            X = X + (vx * cos_yaw - vy * sin_yaw) * dt;
            Y = Y + (vx * sin_yaw + vy * cos_yaw) * dt;
            yaw = yaw + w_yaw * dt;
            yaw = atan2(sin(yaw), cos(yaw));
        
            % calculate new wheel velocities
            p_wheel = obj.data.p_wheel;
            w_wheel = obj.data.w_wheel;
            a_wheel = obj.data.a_wheel;

            w_wheel = w_wheel + a_wheel*dt;
            p_wheel = p_wheel + w_wheel*dt + 0.5*a_wheel*dt^2;
        
            % set
            obj.data.X = X;
            obj.data.Y = Y;
            obj.data.yaw = yaw;
            obj.data.vx = vx;
            obj.data.vy = vy;
            obj.data.w_yaw = w_yaw;
            obj.data.p_wheel = p_wheel;
            obj.data.w_wheel = w_wheel;
        end

        function obj = stepSimulation(obj, dt, steering_input, throttle_input, brake_input)
            obj.input.steering_input = steering_input;
            obj.input.throttle_input = throttle_input;
            obj.input.brake_input = brake_input;
            obj.calcSteeringAngle();
            obj.calcMotorTorque();
            obj.calcBrakeTorque();
            obj.calcTractionTorques();
            obj.calcTireNormalLoads();
            obj.calcWheelSlipsAndForces();
            obj.calcNetForcesAndMoments();
            obj.calcBodyAccelerations();
            obj.calcWheelAccelerations();
            obj.calcNewState(dt);
        end
    end

    methods (Static)
        function deg = RAD2DEG(x)
            deg = x * 180 / pi;
        end
    end
end