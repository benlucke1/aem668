% Author: Benjamin Lucke
% Title: AEM668 P3 - Structural-Mode Analysis and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clc, clear, close all, format compact
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
acplant.InputName = {'\delta_e', '\delta_f'};
acplant.stateName = {'udot', 'alfadot', 'qdot','thetadot','etadot','etaddot'};
acplant.outputName = {'\Delta a_z_{cp}'};

% Display eigenvalues, natural frequencies, and damping ratios for each
% mode
[wn, zeta, p] = damp(acplant);
fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~Part a~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n')
for i = 1:3
    if i == 1
        data = [p(1:2), real(wn(1:2)), real(zeta(1:2))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Phugoid Mode'});
        disp(T)
        fprintf(['\nThe phugoid mode is a lightly damped, ' ...
            'oscillatory low frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    elseif i == 2
        data = [p(3:4), real(wn(3:4)), real(zeta(3:4))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Short Period'});
        disp(T)
        fprintf(['\nThe short period mode is a heavily damped, ' ...
            'oscillatory moderate-frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    elseif i == 3
        data = [p(5:6), real(wn(5:6)), real(zeta(5:6))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'First Structural Mode'});
        disp(T)
        fprintf(['\nThe first structural mode is a lightly damped, ' ...
            'oscillatory high-frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    end
end
clear i dtable

%% Part b
% Define actuators models and input/output names
actuator_bandwidth = 50; % rad/s
A_e = s/(s+actuator_bandwidth);
A_e.InputName = '\delta_e_c';
A_e.OutputName='\delta_e';

A_f = s/(s+actuator_bandwidth);
A_f.InputName = '\delta_f_c';
A_f.OutputName='\delta_f';
A_a = [A_e 0; 0 A_f]; % Actuator plant matrix

% Define aircraft longitudinal plant with actuator inputs
ac_with_actuators = series(A_a, acplant);

% Output frequency response of open loop aircraft plant with actuators
figure('Name','Open Loop Aircraft Frequency Reponse with Actuators')
fig = bodeplot(ac_with_actuators);
grid on

% Compute & comment on elevator's ability to excite modes
[r, ~] = freqresp(ac_with_actuators(1),[wn(1) wn(3) wn(5)]);
r_elevator = abs(r(:));
r_elevator_db = 20*log10(r_elevator);
fprintf(' ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Part b~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n')
fprintf(['The elevator is able to excite the:\n- phugoid mode with a ' ...
    'magnitude of %.2f dB,\n- short period mode with a ' ...
    'magnitude of %.2f dB,\n- first structural mode with a ' ...
    'magnitude of %.2f dB.\n\n'],r_elevator_db(1), r_elevator_db(2), r_elevator_db(3))

% Compute & comment on fin's ability to excite modes
[r, ~] = freqresp(ac_with_actuators(2),[wn(1) wn(3) wn(5)]);
r_fin = abs(r(:));
r_fin_db = 20*log10(r_fin);

fprintf(['\nThe fin is able to excite the:\n- phugoid mode with a ' ...
    'magnitude of %.2f dB,\n- short period mode with a ' ...
    'magnitude of %.2f dB,\n- first structural mode with a ' ...
    'magnitude of %.2f dB.\n\n'],r_fin_db(1), r_fin_db(2), r_fin_db(3))

% Compare excitation ability of elevator and fin at the 3 modes
excitation_ratios = r_elevator./r_fin;
fprintf(['\nThe elevator is able to excite the \n- phugoid mode %.3f times\n' ...
    '- short period mode %.3f times, and' ...
    '\n- first structural mode %.3f times more effectively than the fin.\n']...
    ,excitation_ratios(1), excitation_ratios(2), excitation_ratios(3))

%% Part c
% Define notch filter parameters
structural_wn = wn(end);
zeta_passive = ac.zeta;
zeta_id_passive = 30*zeta_passive;
K_psmc = (s^2 + 2*zeta_passive*structural_wn*s + structural_wn^2)/...
    (s^2 + 2*zeta_id_passive*structural_wn*s + structural_wn^2);
K_psmc.InputName = '\delta_e_c';


% Show root locus of system with notch filter
passive_ac_ol = series(K_psmc, ac_with_actuators(1));
figure('Name','Root Locus for Passive SMC Aircraft')
rlocus(passive_ac_ol)

% Find maximum proportional gain to keep system stable
gains = linspace(0,1,10000);
for k = gains
    idx = find(gains == k);
    passive_ac_cl = feedback(passive_ac_ol,k);
    closest_ol_pole_k = max(real(pole(passive_ac_cl)));
    if closest_ol_pole_k > 0
        clear passive_ac_cl
        max_k_out_passive = gains(idx-2);
        break
    end
end
% Plot closed loop response with various gains
kd1 = feedback(passive_ac_ol,0); % Open Loop
kd4 = feedback(passive_ac_ol,max_k_out_passive/4); % 1/4 of maximum gain
kd2 = feedback(passive_ac_ol,max_k_out_passive/2); % half of maximum gain
k3d4 = feedback(passive_ac_ol,max_k_out_passive*3/4); % 3/4 of maximum gain
k4d5 = feedback(passive_ac_ol,max_k_out_passive*4/5); % 4/5 of maximum gain
figure('Name','Step Response with Passive SMC')
hold on
stepplot(kd1,kd4,kd2,k3d4, k4d5)
legend()
hold off

% Comment on passive SMC
fprintf('\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Part c~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
fprintf(['\nFrom the step response and the root locus plot, the system is ' ...
    'unstable in unity feedback gain.\nHowever, the system is stable in negative feedback with' ...
    ' gain up to %.5f. The step responses show a decrease in settling' ...
    '\ntime and peak acceleration, and an increase in trasient oscillation frequency' ...
    ' as the proportional gain is increased.\nAt the maximum gain, the settling ' ...
    'time and oscillations become very large in magnitude and frequency.\n'],max_k_out_passive)


%% Part d 

% Define frequencies for bandpass and construct active SMC 
offset = 5;
omega_lp = structural_wn + offset;
omega_hp = structural_wn - offset;
highpass_active = s/(s+omega_hp);
lowpass_active = omega_lp/(s+omega_lp);
K_asmc = series(highpass_active,lowpass_active);
K_asmc.InputName = '\delta_f_c';
active_ac_ol = series(K_asmc, ac_with_actuators(2));
figure('Name', 'Root Locus for Active SMC Aircraft')
rlocusplot(active_ac_ol)

gains = linspace(0,1,10000);
for k = gains
    idx = find(gains == k);
    active_ac_cl = feedback(active_ac_ol,k);
    closest_ol_pole_k = max(real(pole(active_ac_cl)));
    if closest_ol_pole_k > 0
        clear active_ac_cl
        max_k_out_active = gains(idx-2);
        break
    end
end

% Plot closed loop response with various gains
kd1 = feedback(active_ac_ol,0); % Open Loop
kd4 = feedback(active_ac_ol,max_k_out_active/4); % 1/4 of maximum gain
kd2 = feedback(active_ac_ol,max_k_out_active/2); % half of maximum gain
k3d4 = feedback(active_ac_ol,max_k_out_active*3/4); % 3/4 of maximum gain
k4d5 = feedback(active_ac_ol,max_k_out_active*4/5); % 4/5 of maximum gain
figure('Name','Step Response with Active SMC')
hold on
stepplot(kd1,kd4,kd2)
legend()
hold off

% Comment on active SMC
fprintf('\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Part d~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
fprintf(['\nFrom the step response and the root locus plot, the system is ' ...
    'unstable in unity feedback gain.\nHowever, the system is stable in negative feedback with' ...
    ' gain up to %.5f. The step responses show an increase in settling' ...
    '\ntime and peak acceleration, and an increase in trasient oscillation frequency' ...
    ' as the proportional gain is increased.\nAt the maximum gain, the settling ' ...
    'time and oscillations become very large in magnitude and frequency.' ...
    '\nHence, for the active SMC, the lowest gain possible will result in the best response.\n'], max_k_out_active)

%% Part e
% Choose 0.5 of max gain for passsive controller, .5 of max gain for ative
% controller
active_smc = feedback(series(K_asmc,ac_with_actuators(1)), max_k_out_active/2);
passive_smc = feedback(series(K_psmc,ac_with_actuators(1)), max_k_out_passive/2);

figure('Name','Aircraft Closed-Loop Frequency Response with Active & Passive SMC')
bodeplot(active_smc, passive_smc)
legend('Active SMC','Passive SMC')

[wn_active, zeta_active, p_active]  = damp(active_smc);
[wn_passive, zeta_passive, p_passive]  = damp(passive_smc);

fprintf('\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Part e~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
fprintf('Passive controller')
K_psmc
fprintf('\n\n')

fprintf('Active controller')
K_asmc
fprintf('\n\n')

% Active Controller Modal Characteristics
fprintf('\n\nActive Controller Modal Characteristics\n')
for i = 1:5
    if i == 1
        data = [p_active(1:2), real(wn_active(1:2)), real(zeta_active(1:2))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Phugoid Mode'});
        disp(T)
        fprintf(['\nThe phugoid mode is a lightly damped, ' ...
            'oscillatory low frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    elseif i == 2
        data = [p_active(3:4), real(wn_active(3:4)), real(zeta_active(3:4))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Short Period'});
        disp(T)
        fprintf(['\nThe short period mode is a heavily damped, ' ...
            'oscillatory moderate-frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    elseif i == 3
        data = [p_active(5:6), real(wn_active(5:6)), real(zeta_active(5:6))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'First Structural Mode'});
        disp(T)
        fprintf(['\nThe first structural mode is a lightly damped, ' ...
            'oscillatory high-frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
     elseif i == 4
        data = [p_active(7:8), real(wn_active(7:8)), real(zeta_active(7:8))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'Actuator Mode'});
        disp(T)
        fprintf(['\nThe actuator  mode is a lightly damped, ' ...
            'moderate-frequency mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
     elseif i == 5
        data = [p_active(9:10), real(wn_active(9:10)), real(zeta_active(9:10))];
        dtable = array2table(data,'VariableNames',...
            {'Eigenvalues' 'Natural Frequency (rad/s)' 'Damping Ratio'});
        T = table(dtable,'VariableNames', {'SMC'});
        disp(T)
        fprintf(['\nThe controller mode is a critically damped ' ...
            'mode that is ' ...
            'stable due to negative complex poles.\n\n\n\n'])
    end
end


fprintf(['The active SMC and the passive SMC both decreased the magnitudes ' ...
    'of the acceleration experienced\nat the cockpit due to the first fuselage' ...
    ' bending mode. \nAdditionally, both ' ...
    'controllers decreased the magnitude of the oscillations at lower ' ...
    'frequencies.\nThe active SMC decreased the magnitude of the modes at ' ...
    'high frequencies, as well.\n'])