clear
close all
clc

addpath('fcn')

%% load configurations
vehicle = Vehicle2D(Vehicle2DConfig('configs/toyota_86.json'));
vehicle.data = toSym(vehicle.data);
vehicle.input = toSym(vehicle.input);

%% assumptions
vehicle = assumeReal(vehicle);

% config
if isa(vehicle.config.gear_ratio, 'sym')
    assume(vehicle.config.gear_ratio > 0)
    assume(vehicle.config.wheelbase > 0)
    assume(vehicle.config.track_width > 0)
    assume(vehicle.config.steer_max > 0)
    assume(vehicle.config.mass > 0)
    assume(vehicle.config.Izz > 0)
    assume(vehicle.config.z_cg > 0)
    assume(vehicle.config.a > 0)
    assume(vehicle.config.a < vehicle.config.wheelbase)
    assume(vehicle.config.r_wheel > 0)
    assume(vehicle.config.I_wheel > 0)
    assume(vehicle.config.Tmax > 0)
    assume(vehicle.config.Tnegmax > 0)
    assume(vehicle.config.Pmax > 0)
    assume(vehicle.config.Pnegmax > 0)
    assume(vehicle.config.gear_ratio > 0)
    assume(vehicle.config.brake_Tmax > 0)
    assume(vehicle.config.brake_bias > 0)
    assume(vehicle.config.diff_damping > 0)
    assume(vehicle.config.tire_config.B > 0)
    assume(vehicle.config.tire_config.C > 0)
    assume(vehicle.config.tire_config.D > 0)
    assume(vehicle.config.tire_config.E > 0)
    assume(vehicle.config.tire_config.f > 0)
    assume(vehicle.config.tire_config.coeff_rr > 0)
    assume(vehicle.config.tire_config.coeff_stiction > 0)
    assume(vehicle.config.CDx > 0)
    assume(vehicle.config.CMz > 0)
    assume(vehicle.config.Af > 0)
    assume(vehicle.config.rho > 0)
end

% data
assume(vehicle.data.w_wheel > 0)
assume(vehicle.data.vx > 0)
assume(vehicle.data.w_yaw > 0)
assume(vehicle.data.steering_angle < pi/2)
assume(vehicle.data.steering_angle > -pi/2)

% inputs
% assume(vehicle.input.throttle_input == 0)
% assume(vehicle.input.brake_input == 0)
% assume(abs(vehicle.input.steering_input) < 1)

%% calculate state derivative
dt = 0.0001;
% vehicle.calcSteeringAngle(); use sym steering_angle 
vehicle.calcTireNormalLoads();
vehicle.calcWheelSlipsAndForces();
vehicle.calcNetForcesAndMoments();
vehicle.calcBodyAccelerations();
vehicle.calcWheelAccelerations();
% vehicle.calcNewState(dt);

%% substitute
v2 = vehicle.copy();

% unnecessary DOF to 0
v2.data = subs2(v2.data, "yaw", 0);
v2.data = subs2(v2.data, "X", 0);
v2.data = subs2(v2.data, "Y", 0);
v2.data = subs2(v2.data, string(v2.data.p_wheel), 0);
v2.data = subs2(v2.data, string(v2.data.wheel_torques), 0);

% simplifications
% ax = -w*vy
% ay = w*vx
v2.data = subs2(v2.data, "ax", -v2.data.w_yaw * v2.data.vy);
v2.data = subs2(v2.data, "ay",  v2.data.w_yaw * v2.data.vx);

% w = -ax/vy = ay/vx
% ay = -ax*vx/vy
% v2.data = subs2(v2.data, "w_yaw", -v2.data.ax / v2.data.vy);
% v2.data = subs2(v2.data, "ay", -v2.data.ax * v2.data.vx / v2.data.vy);

%% analyse
% targets - vx, vy
% control inputs - rear wheel speed, steering angle, yaw rate
% for steady-state:
% d(vx)/dt = (ax + w_yaw * vy) = 0 % subbed
% d(vy)/dt = (ay - w_yaw * vx) = 0 % subbed
% d(w_yaw)/dt = a_yaw = 0
% d(w_wheel1)/dt = a_wheel1 = 0
% d(w_wheel2)/dt = a_wheel2 = 0

syms vx_target vy_target

syms w_wheel_rear
v3 = v2.copy();
v3.data = subs2(v3.data, "vx", vx_target);
v3.data = subs2(v3.data, "vy", vy_target);
v3.data = subs2(v3.data, string(v3.data.w_wheel(3)), w_wheel_rear);
v3.data = subs2(v3.data, string(v3.data.w_wheel(4)), w_wheel_rear);

% solve
clc
clear eqn
eqn(1) = v3.data.a_yaw == 0;
eqn(2) = v3.data.a_wheel(1) == 0;
eqn(3) = v3.data.a_wheel(2) == 0;
eqn(4) = v3.data.Fx - v3.config.mass * (-v3.data.w_yaw * v3.data.vy) == 0;
eqn(5) = v3.data.Fy - v3.config.mass * ( v3.data.w_yaw * v3.data.vx) == 0;
sv = symvar(eqn);
idx_targets = contains(string(sv), "_target");
uv = sv(idx_targets);
sv = sv(~idx_targets);

%% solve
t_vx_vec = linspace(0, 5, 41);
t_vy_vec = linspace(0, 5, 41);
t_vx_vec = t_vx_vec(2:end);
t_vy_vec = t_vy_vec(2:end);
[t_vxs, t_vys] = meshgrid(t_vx_vec, t_vy_vec);
t_vxs = t_vxs(:);
t_vys = t_vys(:);

lastmod = 1;
warning off
for i = 1:length(t_vxs)
    newmod = floor(i / length(t_vxs) * 100);
    if newmod ~= lastmod
        fprintf("%d%%\n", newmod)
    end
    lastmod = newmod;
    % continue

    t_vx = t_vxs(i);
    t_vy = t_vys(i);
    % sub targets
    eqn_vsub = subs(eqn, 'vx_target', t_vx);
    eqn_vsub = subs(eqn_vsub, 'vy_target', t_vy);
    % eqn_vsub = simplify(lhs(eqn_vsub)) == 0;
    
    % estimate initial stuff
    [delta_0slip, w_wheel_front, w_r_kinematic, est_wz] = estimateInitial(t_vx, t_vy, v3.config);
    
    % create matlab function
    f2 = matlabFunction(lhs(eqn_vsub), 'Vars', {'steering_angle', 'w_wheel1', 'w_wheel2', 'w_yaw', 'w_wheel_rear'});
    
    % solve
    % f2v
    % x = [steering angle, 
    %       w_wheel front left, 
    %       w_wheel front right, 
    %       wz,
    %       w_wheel rear (both are equal)
    %       ]
    scale = [1; 100; 100; 1000; 100]';
    f2v = @(x) (f2(x(1), x(2), x(3), x(4), x(5)));
    f2vn = @(x) sum(((f2v(x)./scale)).^2);
    
    if 0
        %% fsolve
        % opts = optimoptions('fsolve', 'MaxFunctionEvaluations', 1e5, 'MaxIterations',1e5);
        % [optx, val] = fsolve(f2v, [delta_0slip, w_wheel_front, est_wz, w_r_kinematic]-0.1, opts)
    else
        %% global search
        x0x = [delta_0slip, w_wheel_front+0.01, est_wz, w_r_kinematic];
        lbx = [-pi/2, w_wheel_front*0.8, -10, w_r_kinematic];
        ubx = [pi/2, (w_wheel_front+1)*1.2, 10, w_r_kinematic*40];
        gs = GlobalSearch;
        gs.NumTrialPoints = 1e4;
        problem = createOptimProblem("fmincon", ...
            x0=x0x, ...
            objective=f2vn, ...
            lb=lbx, ...
            ub=ubx);
        [optx, valn] = run(gs, problem);
        val = f2v(optx);
    end
    
    opt.vx_target = t_vx;
    opt.vy_target = t_vy;
    opt.delta = optx(1);
    opt.w_wheel_front_left = optx(2);
    opt.w_wheel_front_right = optx(3);
    opt.wz = optx(4);
    opt.w_wheel_rear = optx(5);
    opt.score = f2v(optx);

    opts(i) = opt;
end
warning on

%% simulate

%% visualize
% scores = [opts.score];
% scores = reshape(scores, 5, [])'
% plot(scores)
vx = [opts.vx];
vy = [opts.vy];
delta = [opts.delta];
w_wheel_front_left = [opts.w_wheel_front_left];
w_wheel_front_right = [opts.w_wheel_front_right];
wz = [opts.wz];
w_wheel_rear = [opts.w_wheel_rear];
x = unique(vx);
y = unique(vy);

[~, ix] = ismember(vx, x);
[~, iy] = ismember(vy, y);

Z = nan(numel(y), numel(x));
Z(sub2ind(size(Z), iy, ix)) = w_wheel_rear;

surf(x, y, Z);
xlabel('v_x');
ylabel('v_y');
zlabel('\delta');
shading interp;


%% functions
function d = toSym(d)
fns = string(fieldnames(d))';
for fn = fns
    if isscalar(d.(fn))
        d.(fn) = sym(fn);
    else
        d.(fn) = sym(fn, size(d.(fn)));
    end
end
end

function s = assumeReal(s)
fns = string(fieldnames(s))';
for fn = fns
    if isa(s.(fn), 'sym')
        assume(s.(fn), 'real')
    elseif isa(s.(fn), 'struct')
        s.(fn) = assumeReal(s.(fn));
    elseif ~isnumeric(s.(fn))
        try
            fieldnames(s.(fn));
            s.(fn) = assumeReal(s.(fn));
        catch
            error('neither sym nor like-struct nor real')
        end
    end
end
end

function s = subs2(s, sv_strings, vals)
if isscalar(vals)
    iss = true;
elseif length(vals) == length(sv_strings)
    iss = false;
else
    error('length of vals must be equal to 1 or to that of sv_strings')
end
for j = 1:length(sv_strings)
    sv_string = sv_strings(j);
    fns = string(fieldnames(s))';
    for fn = fns
        svs = symvar(s.(fn));
        i = find(string(svs) == sv_string);
        if ~isempty(i)
            sv = svs(i);
            break
        end
    end
    if exist('sv')
        for fni = 1:length(fns)
            fn = fns(fni);
            if iss
                s.(fn) = subs(s.(fn), sv, vals);
            else
                s.(fn) = subs(s.(fn), sv, vals(j));
            end
        end
        clear sv
    end
end
end

function [delta_0slip, w_wheel_front, w_r_kinematic, est_wz] = estimateInitial(t_vx, t_vy, config)
% estimate initial guesses based on target velocities

est_beta = atan2(t_vy, t_vx);
est_v = norm([t_vy, t_vx]);
assumption_latacc = 0.8*9.81 * est_beta;
est_wz = assumption_latacc / est_v;

% calc velocities at wheels
vy_f = t_vy + [ 1 1] * est_wz * config.a;
vx_f = t_vx + [-1 1] * est_wz * config.track_width / 2;

% delta for slip-angle = 0
delta_0slip = mean(atan2(vy_f, vx_f));

% velocities in wheel frame
vx_wheelframe = vx_f .* cos(delta_0slip) + vy_f .* sin(delta_0slip);
% vy_wheelframe = vx_f .* sin(delta_0slip) - vy_f .* cos(delta_0slip);

% rotational rate of wheel for kinematic rolling
w_wheel_front = vx_wheelframe / config.r_wheel;

% nan check
w_wheel_front(isnan(w_wheel_front)) = 0;
delta_0slip(isnan(delta_0slip)) = 0;
est_wz(isnan(est_wz)) = 0;

w_r_kinematic = t_vx / config.r_wheel;

end
