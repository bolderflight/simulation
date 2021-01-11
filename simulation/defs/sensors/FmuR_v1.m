% Configures the UltraStick25e simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% IMU model
Sensors.Imu.sample_rate_hz = 100;
% MPU-9250
% Accel
Sensors.Imu.Accel.scale_factor = [1 0 0;...
                              0 1 0;...
                              0 0 1];
Sensors.Imu.Accel.bias_mps2 = [0 0 0];
Sensors.Imu.Accel.noise_mps2 = 0.0785 * [1 1 1];
Sensors.Imu.Accel.upper_limit_mps2 = 156.9064 * [1 1 1];
Sensors.Imu.Accel.lower_limit_mps2 = -1 * Sensors.Imu.Accel.upper_limit_mps2;
% Gyro
Sensors.Imu.Gyro.scale_factor = [1 0 0;...
                             0 1 0;...
                             0 0 1];
Sensors.Imu.Gyro.bias_radps = [0 0 0];
Sensors.Imu.Gyro.accel_sens_radps = [0 0 0];  % G-sensitivity in rad/s per m/s/s
Sensors.Imu.Gyro.noise_radps = deg2rad(0.1) * [1 1 1];
Sensors.Imu.Gyro.upper_limit_radps = deg2rad(2000) * [1 1 1];
Sensors.Imu.Gyro.lower_limit_radps = -1 * Sensors.Imu.Gyro.upper_limit_radps;
% Magnetometer
Sensors.Imu.mag.sample_rate_hz = 100;
Sensors.Imu.Mag.scale_factor = [1 0 0;...
                             0 1 0;...
                             0 0 1];
Sensors.Imu.Mag.bias_ut = [0 0 0];
Sensors.Imu.Mag.noise_ut =  0.6 * [1 1 1];
Sensors.Imu.Mag.upper_limit_ut =  4800 * [1 1 1];
Sensors.Imu.Mag.lower_limit_ut = -1 * Sensors.Imu.Mag.upper_limit_ut;
%% GNSS model
Sensors.Gnss.sample_rate_hz = 5;
Sensors.Gnss.fix = 3; % 3D fix
Sensors.Gnss.num_satellites = 16;
Sensors.Gnss.pos_accuracy_m = [1.5 1.5 5.5];
Sensors.Gnss.vel_accuracy_mps = 0.05 * [1 1 1];
%% Air data model
% Static pressure
Sensors.StaticPres.sample_rate_hz = 2000;
Sensors.StaticPres.scale_factor = 1;
Sensors.StaticPres.bias_pa = 0;
Sensors.StaticPres.upper_limit_pa = 120000;
Sensors.StaticPres.lower_limit_pa = 70000;
Sensors.StaticPres.noise_pa = 0.01 * (Sensors.StaticPres.upper_limit_pa - Sensors.StaticPres.lower_limit_pa);
% Differential pressure
Sensors.DiffPres.sample_rate_hz = 2000;
Sensors.DiffPres.scale_factor = 1;
Sensors.DiffPres.bias_pa = 0;
Sensors.DiffPres.upper_limit_pa = 1000;
Sensors.DiffPres.lower_limit_pa = 0;
Sensors.DiffPres.noise_pa =  0.02 * (Sensors.DiffPres.upper_limit_pa - Sensors.DiffPres.lower_limit_pa);
