clear, clc, close all

params.sref = 0.44; % ft^2
params.d = 0.75; % ft
params.m = 13.98; % slug
params.iyy = 182.5; % slygs-ft^2
params.g0 = 32.2; % ft/s^2
params.gamma = 1.4; 
params.ps = 973.3; % psf
params.n0 = -0.034; % /deg
params.n10 = -0.3392; % /deg
params.n1m = 0.0565333; % /deg
params.n2 = -0.0094457; % /deg^2
params.n3 = 0.000103; % deg^3
params.m0 = -0.206; % /deg
params.m10 = -0.357; % /deg
params.m1m = 0.136; % /deg
params.m2 = -0.019456; % /deg^2
params.m3 = 0.000215; % /deg^3
params.qbymach = 1/2 * params.gamma * params.ps;