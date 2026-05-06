%% run_Project2_report.m
% Generate all results for Project 2 report
% Requires: Project2.m, ControllerSun.m in the same folder

clear; clc; close all

TEAM = 'Ruize Sun';

%% ------------------------------------------------------------------------
%  0) Physical parameters (match Project2 defaults)
m  = 1;
l1 = 1.5; l2 = 1.0; l3 = 0.2;
J1 = (m/12)*(l2^2 + l3^2);
J2 = (m/12)*(l3^2 + l1^2);
J3 = (m/12)*(l1^2 + l2^2);

fprintf('Moments of inertia:\n J1=%.6f, J2=%.6f, J3=%.6f\n\n',J1,J2,J3);

%% ------------------------------------------------------------------------
%  1) Linearization and controllability at the three equilibria
WE = [1 0 0; 0 1 0; 0 0 1];  % [we1, we2, we3] as columns
eq_names = {'we1=[1;0;0]','we2=[0;1;0]','we3=[0;0;1]'};

results = struct;
for k = 1:3
    we = WE(:,k);
    [A,B] = linearize_rigidbody(we,J1,J2,J3);

    % controllability matrix and rank
    Co = ctrb(A,B);
    r  = rank(Co);

    results(k).we = we;
    results(k).A  = A;
    results(k).B  = B;
    results(k).Co = Co;
    results(k).rank = r;

    fprintf('--- Equilibrium %s ---\n',eq_names{k});
    fprintf('A = \n'); disp(A);
    fprintf('B = \n'); disp(B);
    fprintf('rank(ctrb(A,B)) = %d\n\n', r);
end

%% ------------------------------------------------------------------------
%  2) Pole placement for controllable equilibria (report values)
%     NOTE: ControllerSun.m already places poles internally using data.des_poles.
%     Here we also compute a K for documentation & to report the closed-loop poles.
des_poles = [-2.5 -3.0 -3.5];  % gentle to avoid torque saturation in sim

for k = 1:3
    if results(k).rank == 3
        K = place(results(k).A, results(k).B, des_poles);
        lam = eig(results(k).A - results(k).B*K);
        results(k).K_doc   = K;     % for the write-up
        results(k).polesCL = lam;

        fprintf('Pole placement for %s:\n',eq_names{k});
        fprintf('Desired poles: '); fprintf('%.3f  ', des_poles); fprintf('\n');
        fprintf('Placed K =\n'); disp(K);
        fprintf('Closed-loop eigenvalues =\n'); disp(lam.');
        fprintf('\n');
    else
        fprintf('Equilibrium %s is NOT controllable (rank %d).\n\n', ...
            eq_names{k}, results(k).rank);
    end
end

%% ------------------------------------------------------------------------
%  3) Simulations for the controllable equilibrium
%     We will use the same poles inside ControllerSun.m (you can adjust there).
%     Choose the controllable eq automatically:
ctrl_idx = find([results.rank] == 3, 1, 'first');
if isempty(ctrl_idx)
    error('No controllable equilibrium found with the given parameters.');
end
we_ctrl = WE(:,ctrl_idx).';       % row vector for the reference function
ref_fun = @(t) we_ctrl;           % constant reference

% 3a) Small-error, no disturbance (for "local" linear design demo)
disp('Running SIM 1: small error, no disturbance...');
Project2('ControllerSun', ...
    'team', TEAM, ...
    'diagnostics', true, ...
    'reference', ref_fun, ...
    'initcond', we_ctrl(:) + [ -0.2; 0.05; 0.02 ], ...
    'vecplot', [0 0 1], ...
    'snapshotfile', 'sim1_small_error.pdf', ...
    'datafile',    'sim1_small_error.mat');

% 3b) Moderate error, still within basin (tune if needed)
disp('Running SIM 2: moderate initial error...');
Project2('ControllerSun', ...
    'team', TEAM, ...
    'diagnostics', true, ...
    'reference', ref_fun, ...
    'initcond', we_ctrl(:) + [ 0.5; -0.3; 0.2 ], ...
    'vecplot', [0 0 1], ...
    'snapshotfile', 'sim2_moderate_error.pdf', ...
    'datafile',    'sim2_moderate_error.mat');

% 3c) With unknown disturbance torques
disp('Running SIM 3: disturbance on...');
Project2('ControllerSun', ...
    'team', TEAM, ...
    'diagnostics', true, ...
    'reference', ref_fun, ...
    'initcond', we_ctrl(:) + [ 0.3; -0.2; 0.1 ], ...
    'vecplot', [0 0 1], ...
    'disturbance', true, ...
    'snapshotfile', 'sim3_disturbance.pdf', ...
    'datafile',    'sim3_disturbance.mat');

disp('All sims complete. Snapshots (*.pdf) and data (*.mat) saved.');

%% ------------------------------------------------------------------------
%  4) (Optional) Show a tidy summary table in the Command Window
fprintf('\n===== SUMMARY: Controllability Ranks =====\n');
for k = 1:3
    fprintf('%-16s  rank = %d\n', eq_names{k}, results(k).rank);
end
if isfield(results(ctrl_idx),'polesCL')
    fprintf('\nClosed-loop poles at controllable eq (%s):\n', eq_names{ctrl_idx});
    disp(results(ctrl_idx).polesCL.');
end

%% ------------------------------------------------------------------------
%  Helper function(s)
function [A,B] = linearize_rigidbody(we,J1,J2,J3)
% Linearizes the rigid-body angular rate dynamics at equilibrium we (3x1)
% State: w = [w1; w2; w3]
% Inputs: tau = [tau1; tau2]
% Dynamics (component form):
%   w1dot = ((J2-J3)/J1)*w2*w3 + (1/J1)*tau1
%   w2dot = ((J3-J1)/J2)*w3*w1 + (1/J2)*tau2
%   w3dot = ((J1-J2)/J3)*w1*w2
c1 = (J2 - J3)/J1;
c2 = (J3 - J1)/J2;
c3 = (J1 - J2)/J3;

we = we(:);
A = [ 0,        c1*we(3),  c1*we(2);
      c2*we(3), 0,         c2*we(1);
      c3*we(2), c3*we(1),  0       ];

B = [1/J1, 0;
     0,    1/J2;
     0,    0   ];
end