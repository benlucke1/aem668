% Author: Benjamin Lucke
% Date:   15 September 2024
% Title: AEM668 P1 - Lateral-Directional Stability and Control of Airplane

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
clear, clc, close all, format compact
load('p1params.mat') % Load aircraft parameters 

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

C_lat = [0 0 1 0
    0 0 0 1];

D_lat = [0 0
    0 0];

lat_sys = ss(A_lat, B_lat, C_lat, D_lat);
lat_sys.stateName = {'beta', 'p', 'r', 'phi'};
lat_sys.inputName = {'delta aileron', 'delta rudder'};
lat_sys.outputName = {'\Delta r','\Delta\phi'};

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
    'eigenvalue corresponding with the spiral mode lying in the RHP.\n'])