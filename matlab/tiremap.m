clear all
clc
addpath('fcn')

% load config
config = Vehicle2DConfig("configs/tt02.json");
tc = config.tire_config;

% params
Fz_wheel = 1;
w_wheel = 10;
sas = linspace(-pi/4, pi/4, 1001);
srs = linspace(-0.5, .5, 1001);

% calculate
[slip_angles, slip_ratios] = meshgrid(sas, srs);
[Fx_wheel, Fy_wheel] = tc.calcTireForces(slip_angles, slip_ratios, w_wheel, Fz_wheel);

%% plot
s = subplot(131);
idxp = sqrt(slip_angles.^2 / max(abs(slip_angles), [], 'all') + slip_ratios.^2 / max(abs(slip_ratios), [], 'all')) < 0.707;
Fx_wheel(~idxp) = nan;
Fy_wheel(~idxp) = nan;
surf(slip_angles, slip_ratios, Fx_wheel, 'LineStyle','none')
xlabel('slip angle [rad]')
ylabel('slip ratio [-]')
zlabel('Fx')

s(2) = subplot(132);
surf(slip_angles, slip_ratios, Fy_wheel, 'LineStyle','none')
xlabel('slip angle [rad]')
ylabel('slip ratio [-]')
zlabel('Fy')

linkprop(s, {'CameraUpVector', 'CameraPosition', 'CameraTarget'});

subplot(133)
contourf(slip_angles, slip_ratios, sqrt(Fx_wheel.^2 + Fy_wheel.^2))
xlabel('slip angle [rad]')
ylabel('slip ratio [-]')
zlabel('Fnet')
