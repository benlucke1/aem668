% Author: Benjamin Lucke
% Title: AEM668 P3 - Structural-Mode Analysis and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clc, clear, close all
run VehicleParams.m % Import vehicle parameters and stability derivatives

%% Part a
A_ac = [derivs.xu, derivs.xa, derivs.xq, -conds.g, derivs.xh, derivs.xhdot;
    derivs.zu/conds.ubar, derivs.za/conds.ubar, 1 + derivs.zq/conds.ubar, ...
    0, derivs.zh/conds.ubar, derivs.zhdot/conds.ubar;
    derivs.Mu, derivs.Ma, derivs.Mq, 0, derivs.Mh, derivs.Mhdot;
    0, 0, 1, 0, 0, 0;
    0 0 0 0 0 1;
    derivs.xiu, derivs.xia, derivs.xiq, 0, derivs.xih - (ac.omega3.^2) ...
    derivs.xihdot - 2*ac.zeta.*ac.omega3;
    ];

B_ac = [derivs.xde, derivs.xdf;
        derivs.zde/conds.ubar, derivs.zdf/conds.ubar;
        derivs.Mde derivs.Mdf;
        0 0;
        0 0;
        derivs.xide derivs.xidf];

C_ac = [0 conds.ubar -ac.xcp -conds.ubar 0 ac.vz]*A_ac;
D_ac = [0 conds.ubar -ac.xcp -conds.ubar 0 ac.vz]*B_ac;

acplant = ss(A_ac, B_ac, C_ac, D_ac);
acplant.inputName = {'\delta_e', '\delta_f'};
acplant.stateName = {'udot', 'alfadot', 'qdot','thetadot','etadot','etaddot'};
acplant.outputName = {'\delta a_z_{cp}'};

[wn, zeta, p] = damp(acplant);
for i = 1:3
    if i == 1
        data = [p(1:2), real(wn(1:2)), real(zeta(1:2))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Phugoid Mode'});
        disp(T)
        fprintf(['\nThe phugoid mode is a lightly damped, ' ...
            'oscillatory low frequency mode that is stable.\n\n\n\n'])
    elseif i == 2
        data = [p(3:4), real(wn(3:4)), real(zeta(3:4))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Short Period'});
        disp(T)
        fprintf(['\nThe short period mode is a heavily damped, ' ...
            'oscillatory moderate-frequency mode that is stable.\n\n\n\n'])
    elseif i == 3
        data = [p(5:6), real(wn(5:6)), real(zeta(5:6))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'First Structural Mode'});
        disp(T)
        fprintf(['\nThe first structural mode is a lightly damped, ' ...
            'oscillatory high-frequency mode that is stable.\n\n\n\n'])
    end
end

%% Part b
actuator_bandwidth = 50; % rad/s
A_e = s/(s+actuator_bandwidth);
A_e.InputName = '\delta_e_c';
A_e.OutputName='delta_e';

A_f = s/(s+actuator_bandwidth);
A_f.InputName = 'delta_f_c';
A_f.OutputName='delta_f';

A_a = [A_e 0; 0 A_f];

% System with actuator inputs
ActuatorsAircraftPlant = series(A_a, acplant);

% Output frequency response 
figure(1)
bodeplot(ActuatorsAircraftPlant)
grid on