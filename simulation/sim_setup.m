% Configures and starts simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Add paths
addpath(genpath('vms'));
addpath(genpath('sensor_processing'));
addpath(genpath('models'));

%% Initial conditions
% latitude [deg], longitude [deg], and altitude [m]
initCond.lla_deg_m = [35.148546, -106.732017, 1700];
% Indicated airspeed [m/s]
initCond.ias_mps = 17;

%% Control law configuration
% Control law frame rate and frame period
Config.frameRate_hz = 50;
Config.framePeriod_s = 1 / Config.frameRate_hz;

%% Load data
% Load bus definitions
load('bus_defs.mat');
%% Find trim solution

%% Open model
% bdload
