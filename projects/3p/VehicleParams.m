clear, clc, close all

%% Define aircraft properties and flight conditions
ac.lf = 143; % ft, length of vehicle
ac.lcg = 88.42; % ft, cg location from nose
ac.sw = 1950; % ft^2, wing are
ac.cw = 15.3; % ft, chord length
ac.bw = 70; % ft, wing span
ac.m = 184; % sl-ft^3, mode 3 generalized mass
ac.vz = 0.32; % ft, mode 3 displacement at xcl (xcp)
ac.w = 288000; % lbs, weight @ msl
ac.xcp = 63.4; % ft, cockpit location from cg
ac.ixx = 1.5e5; % sl-ft^2
ac.iyy = 7e6; % sl-ft^2
ac.izz = 7.1e6; % sl-ft^2
ac.omega3 = 12.6; % rad/sec, mode 3 frequency
ac.vzp = -.0027; % rad, mod 3 shape
ac.zeta = 0.01;


conds.hbar = 5000; % ft, altitude of aircraft
conds.ubar = 659; % ft/s, airspeed (true?)
conds.clbar = 0.340; % lift coefficient
conds.mach = 0.6; % mach number
conds.alfabar = 0; % degrees, Angle of attack
conds.cdbar = 0.0285; % drag coefficient
Re = 6.3710088e6;
g0 = 9.80665; 
conds.g = g0*(Re/(Re + conds.hbar));
%% Define Stabiltiy and Control Derivatives
% X Stability and Control Derivatives
derivs.xu = -0.013;
derivs.xa = 19.45;
derivs.xadot = 0;
derivs.xq = -1.913;
derivs.xde = 14.83;
derivs.xdf = 0.742;
derivs.xh = 0;
derivs.xhdot = 0;

% Z Stability and Control Derivatives
derivs.zu = -0.1001;
derivs.za = -283.3;
derivs.zadot = 0;
derivs.zq = -16.55;
derivs.zde = -42.22;
derivs.zdf = 2.11;
derivs.zh = -2.812;
derivs.zhdot = -0.0968;

% M Stability and Control Derivatives
derivs.Mu = 0.003;
derivs.Ma = -3.445;
derivs.Madot = -0.1035;
derivs.Mq = -3.136;
derivs.Mde = -5.346;
derivs.Mdf = 3.376;
derivs.Mh = -0.0663;
derivs.Mhdot = -0.00372;

% Xi Stability and Control Derivatives
derivs.xiu = 0;
derivs.xia = -1075;
derivs.xiadot = 0;
derivs.xiq = -79.44;
derivs.xide = -923.0;
derivs.xidf = 89.53;
derivs.xih = 4.219;
derivs.xihdot = -0.3502;

conds.zeta = 0.01;