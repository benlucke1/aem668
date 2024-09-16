% Author: Benjamin Lucke
% Date:   15 September 2024
% Title: AEM668 P1 - Lateral-Directional Stability and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clear, clc, close all, format compact
load('p1params.mat') % Load aircraft parameters 

% Side note:
% (I hate MATLAB's comment syntax, just make it a # like every other 
% language so I don't have to reach for the 5 key) 

%% 
v_inf = 120 % meters per second
cruise_altitude = 6000; % m 
g = 9.788; % m*s^-2
a_inf = 316.43; % m*s^-1
M_inf = v_inf/a_inf % local velocity/local speed of sound


% lifting line
den_t1 = 4*pi^2 * wing.aspect_ratio^2 * (1-M_inf^2)/(full.coef_lift_slope^2)
den_t2 = 1 + tan(wing.sweep)^2/(1-M_inf^2)

num = 2*pi*wing.aspect_ratio
den = 2 + sqrt(den_t1 * den_t2 + 4)

num/den

