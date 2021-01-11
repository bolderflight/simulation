% Configures the UltraStick25e simulation
%
% Brian R Taylor
% brian.taylor@bolderflight.com
% 
% Copyright (c) 2021 Bolder Flight Systems
%

% Aircraft name
Aircraft.aircraft = 'UltraStick25e';
%% Mass properties
% Mass, kg
Aircraft.Mass.mass_kg = 1.959;
% c.g. location [x y z], m
Aircraft.Mass.cg_m = [0.222 0 0.046];
% Moments of inertia, kg*m^2
Aircraft.Mass.ixx_kgm2 = 0.07151;
Aircraft.Mass.iyy_kgm2 = 0.08636;
Aircraft.Mass.izz_kgm2 = 0.15364;
Aircraft.Mass.ixz_kgm2 = 0.014;
Aircraft.Mass.inertia_kgm2 = [Aircraft.Mass.ixx_kgm2    0   -Aircraft.Mass.ixz_kgm2;...
                        0          Aircraft.Mass.iyy_kgm2          0;...
                        -Aircraft.Mass.ixz_kgm2   0       Aircraft.Mass.izz_kgm2];
%% Geometric parameters
% Chord, m
Aircraft.Geom.c_m = 0.25;
% Wing span, m
Aircraft.Geom.b_m = 1.27;
% Wing area, m^2
Aircraft.Geom.s_m2 = 0.3097;
% Center of pressure [x y z], m
Aircraft.Geom.cp_m = [0.2175 0 0.046];
