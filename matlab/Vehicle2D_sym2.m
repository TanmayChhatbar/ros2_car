clear
close all
clc

addpath('fcn')

%% load configurations
config = Vehicle2DConfig("configs/toyota_86.json");
vehicle = Vehicle2D(config);

%% simulation parameters
dt = 0.001;      % [s]

%% initial
syms throttle_input steering_input
brake_input = sym(0);    % [0 to 1]
vehicle.data.vx = sym(1);

%% assumptions
% assume(vehicle.data.w_wheel(3:4) > 0)
% assume(gear_ratio > 0)
% assume(throttle_input > 0)

%% optimize
% targets - vx, vy
% control inputs - rear wheel speed, steering angle, yaw rate

% from CPP
% csv_data.push_back({"vx_target [m/s]",
%                     "vy_target [m/s]",
%                     "opt_score [-]",
%                     "opt_wheelspeed [rad/s]",
%                     "opt_steering_angle [rad]",
%                     "opt_yawrate [rad/s]"});
x_des = [1, 0];
u0 = [1, pi/4, 0];
lb = [0 -pi/2 0.1];
ub = [1e3 pi/2 5];

% optimize u to make all values in g = 0
f = @(u) getStateGradient(vehicle, x_des, u)

% fmincon
% options = optimoptions('fmincon', 'Display','iter');
% uopt = fmincon(f, u0, [], [], [], [], lb, ub, [], options)
% [~, gv] = getStateGradient(vehicle, x_des, uopt)

% fgoalattain
options = optimoptions('fgoalattain', 'Display','iter', ...
    'StepTolerance', 1e-20, ...
    'FunctionTolerance', 1e-20, ...
    'OptimalityTolerance', 1e-34)
[uopt, gv] = fgoalattain(f, u0, zeros(1, 5), ones(1, 5), ...
    [], [], [], [], [], [], [], options)
% gv = getStateGradient(vehicle, x_des, uopt)

% result
%% functions
% function [g, gv] = getStateGradient(vehicle, x, u)
function gv = getStateGradient(vehicle, x, u)
vehicle.data.vx = x(1);
vehicle.data.vy = x(2);

w_rear_wheels = u(1);
steering_angle = u(2);
yaw_rate = u(3);

% vehicle.input.throttle_input = throttle_input;
% vehicle.input.brake_input = brake_input;
vehicle.input.throttle_input = 0;
vehicle.input.brake_input = 0;
% vehicle.calcSteeringAngle(); % apply steering_input
vehicle.data.steering_angle = steering_angle;
vehicle.data.w_wheel(3:4) = w_rear_wheels; % apply w_rear_wheels
vehicle.data.w_yaw = yaw_rate; % apply yaw rate
% vehicle.calcMotorTorque();
% vehicle.calcBrakeTorque();
% vehicle.calcTractionTorques();
vehicle.data.motor_torque = 0;
vehicle.data.brake_torque = zeros(1, 4);
vehicle.data.wheel_torques = zeros(1, 4);
vehicle.calcTireNormalLoads();
w_front_wheels_prev = [-inf -inf];
n = 0;
dtt = 0.001;
while any(abs(w_front_wheels_prev - vehicle.data.w_wheel(1:2)) > 1e-3)
    if n > 200
        dtt = 0.00001;
    elseif n > 20
        dtt = 0.0001;
    end
    w_front_wheels_prev = vehicle.data.w_wheel(1:2);
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcWheelAccelerations();
    vehicle.data.w_wheel(1:2) = ...
        vehicle.data.w_wheel(1:2) + vehicle.data.a_wheel(1:2) * dtt;
    n = n + 1;
end
vehicle.calcNetForcesAndMoments();
vehicle.calcBodyAccelerations();

ax = vehicle.data.ax;
ay = vehicle.data.ay;
w_yaw = vehicle.data.w_yaw;
vx = vehicle.data.vx;
vy = vehicle.data.vy;
a_yaw = vehicle.data.a_yaw;
gv = double([(ax + w_yaw * vy); % d(vx) / dt
    (ay - w_yaw * vx);  % d(vy) / dt
    a_yaw;              % d(w_yaw) / dt
    vehicle.data.a_wheel(1:2)'; ... % change of front wheel speeds
    ]);
% disp(gv')
g = sum(gv.^2);
end