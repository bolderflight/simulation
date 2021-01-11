% Configures and starts simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

%% Cleanup
bdclose all;
close all;
clear all;
clc;

%% Target trim conditions


%% Definitions
% Vehicle
vehicle = 'UltraStick25e';
% Sensors
sensor = 'FmuR_v1';
% Actuators
actuator = 'empty';
% Motor
motor = 'empty';
% Propeller
prop = 'empty';

%% Add paths
addpath(genpath('vms'));
addpath(genpath('sensor_processing'));
addpath(genpath('models'));
addpath(genpath('defs'));

%% Load bus definitions
load('bus_defs.mat');

%% Call the setup scripts
fh_vehicle = str2func(vehicle);
fh_vehicle();
fh_sensor = str2func(sensor);
fh_sensor();
fh_act = str2func(actuator);
fh_act();
fh_motor = str2func(motor);
fh_motor();
fh_prop = str2func(prop);
fh_prop();

%% Trim

%% Cleanup
clear vehicle sensor actuator motor prop fh_vehicle fh_sensor fh_act ...
    fh_motor fh_prop;
