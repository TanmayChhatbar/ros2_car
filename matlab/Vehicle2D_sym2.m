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
u0 = [1, pi/4, 0]; % wheel speed, steering angle, yaw rate
lb = [0 -2*pi/3  -5];
ub = [1e3 2*pi/3 5];

% optimize u to make all values in g = 0
getStateGradient(vehicle, x_des, u0)
fv = @(u) getStateGradient(vehicle, x_des, u);
f = @(u) getStateGradient(vehicle, x_des, u, true);

% fmincon
% options = optimoptions('fmincon', 'Display','iter');
% uopt = fmincon(f, u0, [], [], [], [], lb, ub, [], options)
% gv = getStateGradient(vehicle, x_des, uopt)

% globalsearch
opts = optimoptions(@fmincon, 'Algorithm','sqp', 'Display', 'iter');
problem = createOptimProblem('fmincon','objective',...
    f,'x0',u0,'lb',lb,'ub',ub,'options',opts);
gs = GlobalSearch;
[uopt,f] = run(gs,problem)
gv = getStateGradient(vehicle, x_des, uopt)


% fgoalattain
% options = optimoptions('fgoalattain', 'Display','iter', ...
%     'StepTolerance', 1e-20, ...
%     'FunctionTolerance', 1e-20, ...
%     'OptimalityTolerance', 1e-34)
% [uopt, gv] = fgoalattain(fv, u0, zeros(1, 5), ones(1, 5), ...
%     [], [], [], [], lb, ub, [], options)

% opts = optimoptions(@fmincon, 'UseParallel', true);
% problem = createOptimProblem('fmincon','objective',...
%     f,'x0',u0,'lb',lb,'ub',ub);
% ms = MultiStart;
% [uopt,f] = run(ms,problem, 200)
% gv = getStateGradient(vehicle, x_des, uopt)

% result
%% functions
function gv = getStateGradient(vehicle, x, u, scalar_out)
if nargin < 4
    scalar_out = false;
end

vehicle.data.vx = x(1);
vehicle.data.vy = x(2);

w_rear_wheels = u(1);
steering_angle = u(2);
yaw_rate = u(3);

% zero vehicle inputs
vehicle.input.throttle_input = 0;
vehicle.input.brake_input = 0;

% apply optimization-control inputs
vehicle.data.steering_angle = steering_angle;
vehicle.data.w_wheel(3:4) = w_rear_wheels;
vehicle.data.w_yaw = yaw_rate;

% wheel torques = 0
vehicle.data.motor_torque = 0;
vehicle.data.brake_torque = zeros(1, 4);
vehicle.data.wheel_torques = zeros(1, 4);
a_front_wheels_prev = [-inf -inf];
vehicle.data.a_wheel(1:2) = inf;
n = 0;
dtt = 0.001;
while any(abs(vehicle.data.a_wheel(1:2)) > 1) && n < 1e4
    if any(sign(a_front_wheels_prev) .* sign(vehicle.data.a_wheel(1:2)) == -1)
        dtt = dtt / 10;
        if dtt < 0.000001
            break
        end
    end
    a_front_wheels_prev = vehicle.data.a_wheel(1:2);
    vehicle.calcTireNormalLoads();
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

if scalar_out
    gv = sum(gv.^2);
end
end