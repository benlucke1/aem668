% Author: Benjamin Lucke
% Date:   15 September 2024
% Title: AEM668 P1 - Lateral-Directional Stability and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clear, clc, close all, format compact
run('project1_parameters.m')
%% Part a - Compute and Output Estimated Stability Derivatives
%%% A Matrix Stability Derivatives
stab_derivatives.cyb = -vtail.eff * vtail.area / wing.area ...
    * vtail.coef_lift_slope * (1 + vtail.sidewash_slope);

stab_derivatives.cyp = (wing.aspect_ratio + cos(wing.sweep))...
    / (wing.aspect_ratio + 4*cos(wing.sweep)) ...
    * tan(wing.sweep) * trim.coef_lift;

stab_derivatives.cyr = 2 * vtail.x / wing.span * stab_derivatives.cyb;

stab_derivatives.clb = wing.coef_l_dihedral * wing.dihedral;

stab_derivatives.clp = -(1 + 3 * wing.taper_ratio)/(1 + wing.taper_ratio)...
    * wing.coef_lift_slope / 12;

stab_derivatives.clr = (trim.coef_lift/4) ...
    - (2*(vtail.x * vtail.z / (wing.span^2)) * stab_derivatives.cyb);

stab_derivatives.cnb = vtail.eff * vtail.volume_ratio ...
    * vtail.coef_lift_slope * (1 + vtail.sidewash_slope);

stab_derivatives.cnp = -trim.coef_lift / 8;

stab_derivatives.cnr = 2 * vtail.eff * vtail.volume_ratio ...
    * vtail.x/wing.span * vtail.coef_lift_slope;

%%% A Matrix Coefficients
y_b = trim.dynamic_pressure * wing.area * stab_derivatives.cyb / full.mass;

y_p = (trim.dynamic_pressure * wing.area * wing.span) ...
    / (2 * full.mass * trim.airspeed) * stab_derivatives.cyp;

y_r = trim.dynamic_pressure * wing.area * wing.span ...
    * stab_derivatives.cyr / (2 * full.mass * trim.airspeed);

l_b = trim.dynamic_pressure * wing.area * wing.span * stab_derivatives.clb ...
    / full.inertia_x;

l_p = trim.dynamic_pressure * wing.area * (wing.span^2) ...
    * stab_derivatives.clp / (2 * full.inertia_x * trim.airspeed);

l_r = trim.dynamic_pressure * wing.area * (wing.span^2) ...
    * stab_derivatives.clr / (2 * full.inertia_x * trim.airspeed);

n_b = trim.dynamic_pressure * wing.area * wing.span ...
    * stab_derivatives.cnb / full.inertia_z;

n_p = trim.dynamic_pressure * wing.area * (wing.span^2) ...
    * stab_derivatives.cnp / (2 * full.inertia_z * trim.airspeed);

n_r = trim.dynamic_pressure * wing.area * (wing.span.^2) ...
    * stab_derivatives.cnr / (2 * full.inertia_z * trim.airspeed);


%%% B Matrix Stabiltiy Derivatives
% Anonymous function to compute integrand of C_l_da
cl_da_integrand = @(y) ((2*wing.coef_lift_slope*wing.ail_tau*wing.root_chord)...
    /(wing.area * wing.span)) * ((y.^2/2) ...
    + (((wing.tip_chord/wing.root_chord) - 1)/(wing.span/2)) * (y.^3/3));

stab_derivatives.cy_dr = vtail.area / wing.area * vtail.rudder_tau ...
    * vtail.coef_lift_slope;
stab_derivatives.cl_da = cl_da_integrand(wing.ail_outer) - cl_da_integrand(wing.ail_inner);
stab_derivatives.cl_dr = vtail.area/wing.area * vtail.rudder_tau * vtail.z/wing.span * vtail.coef_lift_slope;
stab_derivatives.cn_da = 2 * wing.ail_factor * trim.coef_lift * stab_derivatives.cl_da;
stab_derivatives.cn_dr = -vtail.eff * vtail.volume_ratio * vtail.coef_lift_slope * vtail.rudder_tau;


%%% B Matrix Coefficients
y_dr = trim.dynamic_pressure * wing.area / full.mass ...
    * stab_derivatives.cy_dr;

l_da = trim.dynamic_pressure * wing.area * wing.span * stab_derivatives.cl_da / full.inertia_x;

l_dr = trim.dynamic_pressure * wing.area * wing.span * stab_derivatives.cl_dr / full.inertia_x;

n_da = trim.dynamic_pressure * wing.area * wing.span * stab_derivatives.cn_da / full.inertia_z;

n_dr = trim.dynamic_pressure * wing.area * wing.span * stab_derivatives.cn_dr / full.inertia_z;


%%% Define State Space Model
A_lat = [y_b/trim.airspeed, y_p/trim.airspeed, y_r/trim.airspeed - 1, trim.gravity/trim.airspeed*cos(trim.flight_path);
    l_b, l_p, l_r, 0;
    n_b, n_p, n_r, 0;
    0, 1, 0, 0];

B_lat = [0, y_dr/trim.airspeed;
    l_da, l_dr;
    n_da, n_dr;
    0, 0];

C_lat = [0 0 1 0;
    0 0  0 1];

lat_sys = ss(A_lat, B_lat, C_lat, 0);
lat_sys.stateName = {'beta', 'p', 'r', 'phi'};
lat_sys.inputName = {'delta_aileron', 'delta_rudder'};
lat_sys.outputName = {'rdot','phidot'};
size(lat_sys)

%% Part b - Compute and Display Damping of Aircraft Modes
[wn, zeta, p] = damp(lat_sys);
fprintf('___________________________________________________________')
fprintf('\nSpiral Mode')
fprintf('\n\nEigenvalue: %d', p(1))
fprintf('\nNatural Frequency: %f rad/sec', wn(1))
fprintf('\nDamping Ratio: N/A')
fprintf('\n')

fprintf('\n___________________________________________________________')
fprintf('\nRoll Subsistence Mode')
fprintf('\n\nEigenvalue: %d', p(2))
fprintf('\nNatural Frequency: %f rad/sec', wn(2))
fprintf('\nDamping Ratio: N/A')
fprintf('\n')

fprintf('\n___________________________________________________________')
fprintf('\nDutch Roll Mode')
fprintf('\n\nEigenvalues: %f%+fj, %f%-fj', real(p(3)), imag(p(3)), real(p(4)), imag(p(4)))
fprintf('\nNatural Frequency: %f rad/sec', wn(3))
fprintf('\nDamping Ratio: %f', zeta(3))
fprintf('\n')

fprintf(['\nThe aircraft is laterally-directionally unstable due to the\n' ...
    'eigenvalue corresponding with the spiral mode lying in the RHP.\n\n'])

%% Part c - Lateral-directional Heading Hold Guidance System

%%% Uncompensated & Proportional Boost Stage
s = tf('s');
ol = trim.gravity / (s*trim.airspeed);

% Want crossover frequency at 1.2 radians per second
bw = 1.2;
r1 = frd(1,bw);

% For 45 degrees phase margin, want -30 db/dec around crossover frequency
mid_freq = linspace(bw/3, bw*3, 500);
r2 = frd((bw./mid_freq).^(1.5), mid_freq);

% For low frequency response up to .05 radians per sec,
lf = linspace(1e-3, .05);
r3 = frd(tf(1 + 1/.05), lf);

% For high frequency response greater than 10 rad/sec,
hf = linspace(10,1000);
r4 = frd(tf(.05 / (1+.05)), hf);

% Define proportional controller to establish 1.2 rad/s crossover freq.
controller_1 = 1/abs(freqresp(ol, bw));
L1 = controller_1 * ol;

% Compare Open Loop Plant to Requirements
figure()
bodemag(ol,'b', L1, 'b--', r2,'g', r3,'g', r4,'g', r1,'rx')
title('Heading Hold Bode Magnitude of Plant & Requirements')
legend('Uncompensated Plant','Plant with Proportional Boost'...
    ,'Requirements','Interpreter','Latex','FontSize',14)

%%% Integral Stage (this is overkill but I wanted to see if it would work)
integral_boost_stage = (s+.07)/s;
controller_2 = controller_1 * integral_boost_stage;
L2 = ol * controller_2;

% Compare low-frequency gain to requirements
figure()
bodemag(ol, 'b', L2,'b--', r2,'g', r3,'g', r4,'g', r1,'rx')
title('Heading Hold Bode Magnitude of Plant & Requirements')
legend('Uncompensated Plant','Plant with Proportional and Integral Boost'...
    ,'Requirements','Interpreter','Latex','FontSize',14)


%%% Rolloff Stage/Butteworth Filter
wlp = bw*2; % Found by experimenting
rolloff_stage = 1/(1/wlp*s + 1);
controller_3 = controller_2 * rolloff_stage;
L3 = ol * controller_3;
figure()
bodemag(ol, 'b', L3,'b--', r2,'g', r3,'g', r4,'g', r1,'rx')
title('Heading Hold Bode Magnitude of Plant & Requirements')
legend('Uncompensated Plant','Plant with Proportional, Integral Boost and Rolloff'...
    ,'Requirements','Interpreter','Latex','FontSize',14)
closed_loop = feedback(controller_3, 1);


%%% Print stuff for JDL 
fprintf('The heading hold open loop TF is\n')
L3

fprintf(['The poles of the heading hold outer loop are s = %.3f and \n ' ...
    's = %.3f.\n\n'],pole(closed_loop))

[Gm,Pm,Wcg,Wcp] = margin(L3);
fprintf(['The gain margin is infinite. The phase margin is ' ...
    '%.2f degrees\nat a crossover frequency of %.2f rad/sec.\n\n'],Pm,Wcp)

%%% Prove 5% steady state error requirement met
t = linspace(0,400,1000);
input_frequency = 0.05; % rad/sec
u = sin(input_frequency * t);
y_out = lsim(closed_loop, u, t);
e = zeros(1,numel(t));
for i = 1:numel(t)
    e(i) = y_out(i) - u(i);
end
figure()
hold on
plot(t,e)
plot(t, .05*ones(length(t)),'r--', t, -.05*ones(length(t)),'r--')
xlabel('Time (sec)')
ylabel('Error')
title('Outer Loop Tracking Error for u = $sin(.05 t)$','Interpreter',...
    'latex','FontSize',14)
ylim([-.1 .1])
legend('Tracking Error', '5\% Bound','Interpreter','Latex',...
    'Fontsize', 14)
hold off

%%% Prove < 5% gain for frequencies above 10 rad/sec
fprintf(['The magnitude of the open loop response at 10 radians per' ...
    ' second is %.2f dB.\n\n'], 20*log10(abs(freqresp(L3, 10))))

%% Part d - Lateral-directional Inner Loop Controller 
omega_w = 1.820497/10;
tau_actuator = .01;
omega_actuator = 1/tau_actuator;

K_ari = tf(-n_da/n_dr, 'InputName','delta_ac','OutputName', 'delta_ari');

A_a = 1/(s + omega_actuator);
A_a.InputName = 'delta_ac';
A_a.OutputName='delta_aileron';

A_r = 1/(s + omega_actuator);
A_r.InputName = 'delta_rc';
A_r.OutputName='delta_rudder';

K_r = -s/(s+(omega_w/5));
K_r.InputName = 'rdot';
K_r.OutputName = 'rdot_damped';

lat_sys.stateName = {'betadot', 'pdot', 'rdot', 'phidot'};
lat_sys.inputName = {'delta_aileron', 'delta_rudder'};
lat_sys.outputName = {'rdot','phidot'};

actuator_matrix = [A_a 0; 0 A_r];
roll_ari = [1; K_ari];
actuatorsPlusPlant = series(actuator_matrix, lat_sys);
ol_sys = feedback(actuatorsPlusPlant, K_r, 2, 1) * roll_ari;
roll_ol = tf(ol_sys(2));


% Want crossover frequency at 10 radians per second
bw = 10;
r1 = frd(1,bw);

% For 45 degrees phase margin, want -30 db/dec around crossover frequency
mid_freq = linspace(bw/3, bw*3, 500);
r2 = frd((bw./mid_freq).^(1.5), mid_freq);

% For low frequency response up to 1 radians per sec,
lf = linspace(1e-3, 1);
r3 = frd(tf(1 + 1/.05), lf);

% For high frequency response greater than 10 rad/sec < 5% gain,
hf = linspace(100,1000);
r4 = frd(tf(.05 / (1+.05)), hf);

%%% Proportional Control Stage
P_controller = tf(1/abs(freqresp(roll_ol, bw)));


%%% Derivative control stage
wd = 20;
D_controller = tf(1/wd*s + 1);

%%% Integral Control Stage
I_controller = (s+1.820497/2)/s;

%%% Fully compensated system
PID_controller = P_controller*I_controller*D_controller;
L = PID_controller * roll_ol;
roll_cl = feedback(PID_controller * roll_ol, 1);


%%% Print stuff for JDL 
fprintf('The roll control controller TF is \n')
PID_controller

fprintf('The poles of the roll control inner loop loop are\n')
pole(roll_cl)

[Gm,Pm,Wcg,Wcp] = margin(roll_cl);
fprintf(['The gain margin for the roll controller is infinite. The phase margin is ' ...
    '%.2f degrees\nat a crossover frequency of %.2f rad/sec.\n\n'],Pm,Wcp)

fprintf(['The gain at the desired loop bandwidth of %.2f rad/sec is %.4f dB. \nThe gain at ' ...
    '100 rad/sec is %.4f dB.\n'], 11, 20*log10(abs(freqresp(L, 11))), ...
    20*log10(abs(freqresp(L, 100))))

%%% Prove 5% steady state error requirement met
t = linspace(0,400,1000);
input_frequency = 1; % rad/sec
u = sin(input_frequency * t);
y_out = lsim(roll_cl, u, t);
e = zeros(1,numel(t));
for i = 1:numel(t)
    e(i) = y_out(i) - u(i);
end
figure()
hold on
plot(t,e)
plot(t, .05*ones(length(t)),'r--', t, -.05*ones(length(t)),'r--')
xlabel('Time (sec)')
ylabel('Error')
title('Inner Loop Tracking Error for u = $sin(t)$','Interpreter',...
    'latex','FontSize',14)
ylim([-.1 .1])
legend('Tracking Error', '5\% Bound','Interpreter','Latex',...
    'Fontsize', 14)
hold off


simulink_sim = sim('aem668_project1_simulink');