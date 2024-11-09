% Author: Benjamin Lucke
% Date:   6 November 2024
% Title: AEM668 P4 - MIMO LTI Feedback Control of Tail-Controlled Rocket
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc, close all 
run ModelParams
%% 


[da, dq, dd] = partialCn(1,3, params)                



c = soundSpeed(20000,params)

function [dcndalfa, dcndq, dcnddelta] = partialCn(alfa, Mach, params)
% Computes d(Cn)/d(alfa), d(Cn)/d(q), and d(Cn)/d(delta) 
% Partial derivatives of Cn evaluated with Python
if alfa > 0
    dcndalfa = 2*alfa*params.n2 + params.n3*sign(alfa) + (Mach*params.n1m + params.n10)*sign(alfa);
    dcnddelta = params.n0;
elseif alfa < 0
    dcndalfa = -2*alfa*params.n2 - params.n3*sign(alfa) + (-Mach*params.n1m - params.n10)*sign(alfa);
    dcnddelta = -params.n0;
elseif alfa == 0
    dcndalfa = 0;
    dcnddelta = -params.n0;
end
dcndq = 0;
end


function [T, rho] = standardAtmosphere(altitude)
    T = 518.69 - (3.5662e-3)*altitude;
    rho = (6.6277e-15)*T.^4.256;
end

function a = soundSpeed(altitude,params)
    [Tlocal, ~] = standardAtmosphere(altitude)
    a = sqrt(params.gamma * 1716.5 * Tlocal);
end
