clear
close all
clc

addpath('fcn')

% load configurations
config = Vehicle2DConfig("configs/toyota_86.json");
vehicle = Vehicle2D(config);
cruise_control = VehicleCruiseControl(10, 1, 0.0, 0.5, 0.0001);

% prepare output CSV file
CSVWriter = Vehicle2DCSVWriter("output/vehicle_sim_test.csv");

%% simulation parameters
max_steering_input = 0.1;
dt = 0.01;      % [s]
tmax = 2000.0;      % [s]
freq_steer = 0.1; % frequency of steering oscillation
t_start = 1;
f_log = 100;      % [hz] log frequency

%% initial
current_time = 0.0;
step_count = 0;
throttle_input = 1.0; % [-1 to 1]
steering_input = 0.0; % [-1 to 1]
brake_input = 0.0;    % [0 to 1]
vehicle.data.vx = 1;

%% run simulation
fprintf("Running test...\n");
time = 0;
for step = 0:(tmax/dt)
    % disp(step / (tmax/dt))
    % calculate inputs based on current times
    if (current_time > t_start && current_time < t_start + 2 / freq_steer)
        steering_input = max_steering_input * sin(freq_steer * (current_time - t_start) * 2 * pi);
    else
        steering_input = 0.0;
    end

    % cruise control
    w_wheel = vehicle.data.w_wheel;
    vx_est = mean(w_wheel(3:4)) * vehicle.config.r_wheel;
    [cruise_control, throttle_input, brake_input] = cruise_control.calcCruiseControlInputs(vx_est);

    % brake test
    % if (current_time > t_start + 3 && current_time < t_start + 3.5)
    %     brake_input = 1.0;
    % else
    %     brake_input = 0.0;

    % set inputs and step simulation
    vehicle.stepSimulation(dt, steering_input, throttle_input, brake_input);

    % output data
    time = time + dt;
    if mod(step_count, floor(1 / (f_log * dt))) == 0
        CSVWriter.WriteCSVData(time, vehicle);
    end

    current_time = current_time + dt;
    step_count = step_count + 1;
end

CSVWriter.close();
fprintf("Finished test. Data saved to "+filename+"\n");
