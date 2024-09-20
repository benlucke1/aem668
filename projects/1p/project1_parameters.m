%% project1_solution.m

%% Setup workspace.

close all;
clear all;
clc;


%% Set conversion constants.

RAD2DEG = 180/pi;
DEG2RAD = pi/180;


%% Set trim, geometric, mass, and aerodynamic parameters.
% All units are in meters, kilograms, Newtons, radians, and seconds.

% Calculate trim parameters.
trim.altitude = 6000;
trim.density = 0.66011;
trim.gravity = 9.788;
trim.speed_sound = 316.43;
trim.airspeed = 120;
trim.dynamic_pressure = 0.5*trim.density*trim.airspeed^2;
trim.mach = trim.airspeed/trim.speed_sound;
trim.flight_path = 0;
trim.coef_lift = 0.2984;

% Calculate the airplane parameters for the full vehicle.
full.coef_lift_0 = 0.01;
full.coef_drag_0 = 0.036;
full.coef_m_0 = 0.04;
full.coef_lift_slope = 5.05;
full.mass = 5800;
full.inertia_x = 112000;
full.inertia_y = 93000;
full.inertia_z = 194000;
full.gravity_center = 7.5;
full.aero_center = 6.8;

% Calculate wing parameters.
wing.area = 40;
wing.span = 20;
wing.tip_chord = 1;
wing.root_chord = 3;
wing.mean_chord = 2;
wing.taper_ratio = wing.tip_chord/wing.root_chord;
wing.x = full.gravity_center - full.aero_center;
wing.coef_lift_slope = 4.95;
wing.eff = 0.8;
wing.dihedral = 0.04;
wing.sweep = 0.1;
wing.coef_l_dihedral = -0.7;
wing.aspect_ratio = wing.span^2/wing.area;
wing.chord_ratio = wing.tip_chord/wing.root_chord;
wing.ail_inner = 7.5;
wing.ail_outer = 9.5;
wing.ail_factor = -0.1;
wing.ail_area = 0.35;
wing.ail_tau = interp1(0:0.05:0.7, [0, 0.16, 0.26, 0.34, 0.41, ...
    0.47, 0.52, 0.56, 0.60, 0.64, 0.68, 0.72, 0.75, 0.78, 0.8], ...
    wing.ail_area/wing.area);

% Calculate vertical tail parameters.
vtail.area = 5;
vtail.span = 2.5;
vtail.mean_chord = 2;
vtail.x = -8.5;
vtail.z = -0.8;
vtail.coef_lift_slope = 3;
vtail.eff = 0.95;
vtail.sidewash_slope = 0.1;
vtail.volume_ratio = -vtail.x*vtail.area/(wing.area*wing.span);
vtail.rudder_area = 1.2;
vtail.rudder_tau = interp1(0:0.05:0.7, [0, 0.16, 0.26, 0.34, 0.41, ...
    0.47, 0.52, 0.56, 0.60, 0.64, 0.68, 0.72, 0.75, 0.78, 0.8], ...
    vtail.rudder_area/vtail.area);

save('p1params.mat')