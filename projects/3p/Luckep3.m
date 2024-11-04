% Author: Benjamin Lucke
% Title: AEM668 P3 - Structural-Mode Analysis and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clc, clear, close all
run VehicleParams.m % Import vehicle parameters and stability derivatives

%%
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

C_ac = eye(6);
D_ac = 0;

sys = ss(A_ac, B_ac, C_ac, 0);
sys.inputName = {'\delta_e', '\delta_f'};
sys.outputName = {'udot','alfadot','qdot','thetadot','etadot','etaddot'};
[wn, zeta, p] = damp(sys);
tc = abs(ones(1,length(p))'./real(p))
for i = 1:3
    if i == 1
        data = [p(1:2), real(wn(1:2)), real(zeta(1:2))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Phugoid Mode'});
        disp(T)
        fprintf(['\nThe phugoid mode is a lightly damped, ' ...
            'oscillatory low frequency mode that is stable\nwith a large time constant of approximately %.2f seconds.\n\n\n\n'],tc(1))
    elseif i == 2
        data = [p(3:4), real(wn(3:4)), real(zeta(3:4))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Short Period'});
        disp(T)
        fprintf(['\nThe short period mode is a heavily damped, ' ...
            'oscillatory moderate-frequency mode that is stable\nwith a time constant of approximately %.2f seconds.\n\n\n\n'],tc(3))
    elseif i == 3
        data = [p(5:6), real(wn(5:6)), real(zeta(5:6))];
        dtable = array2table(data,'VariableNames',{'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Structural Mode'});
        disp(T)
        fprintf(['\nThe first structural mode is a lightly damped, ' ...
            'oscillatory high-frequency mode that is stable\nwith a time constant of approximately %.2f seconds.\n\n\n\n'],tc(5))
    end
end

step(sys)