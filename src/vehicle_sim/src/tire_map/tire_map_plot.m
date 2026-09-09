% clear all
% close all
clc

filename = "tire_map.csv";
tm = readtable(filename);
slip_angles = tm.slip_angle_rad_;
slip_ratios = tm.slip_ratio___;
Fxs = tm.Fx_N_;
Fys = tm.Fy_N_;

num_rows = length(unique(slip_angles));
slip_angles = rad2deg(reshape(slip_angles, num_rows, []));
slip_ratios = reshape(slip_ratios, num_rows, []);
Fxs = reshape(Fxs, num_rows, []);
Fys = -reshape(Fys, num_rows, []);

%% plot
s(1) = subplot(131);
surf(slip_angles, slip_ratios, Fxs, 'linestyle', 'none')
xlabel("slip angle [deg]")
ylabel("slip ratio [-]")
zlabel("Fx [N]")

s(2) = subplot(132);
surf(slip_angles, slip_ratios, Fys, 'linestyle', 'none')
xlabel("slip angle [deg]")
ylabel("slip ratio [-]")
zlabel("Fy [N]")

s(3) = subplot(133);
surf(slip_angles, slip_ratios, sqrt(Fxs.^2 + Fys.^2), 'linestyle', 'none')
xlabel("slip angle [deg]")
ylabel("slip ratio [-]")
zlabel("Fy [N]")

fontsize(24, 'points')