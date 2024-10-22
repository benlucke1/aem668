% Author: Benjamin Lucke
% Date:   16 Octoner 2024
% Title: AEM668 P2 - Nonlinear Performance Simulation of Subsonic Airplane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clear, clc, close all, format compact
%%
params.Kf = 4e-6; % sl/(lb-s)
params.wt = 2; % rad/sec
params.wl = 2.5;  % rad/sec
params.wu = 1; % rad/sec
params.cd0 = 0.0183;
params.Tmax = 72e3; % lb
params.Sw = 1745; % ft^2
params.Klmax = 2.6; % lb-s^2/ft^2
params.cla = .0920; % /deg
params.AR = 10.1; % dimensionless
params.umax = 30; % deg
params.a0 = -.05; % deg
params.oswald = 0.613;
params.RE = 6371000 * 3.28084; % ft 

FPS2MPH = 3600/5280;
DEG2RAD = pi/180;
RAD2DEG = 180/pi;

ic.vbn = 600; % ft/s
ic.fp0 = 0; % deg
ic.sig0 = 0; % deg
ic.h0 = 20e3; %ft
ic.l0 = 33.2098; % 
ic.lam0 = -87.5692;
ic.vwind = [40, 40, 0];
ic.g0 = 32.17; % ft/s^2
ic.rho = 2.3769e-3;
ic.mg0 = 200e3;

gd.Ktp = .08;
gd.Kti = .002;
gd.Klp = 0.5;
gd.Kli = .01;
gd.Kup = 0.075;