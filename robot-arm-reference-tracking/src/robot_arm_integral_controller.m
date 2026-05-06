%
% STEP #1: Rename this file by replacing "LastName" with your last name.
%

%
% STEP #2: Rename, but do NOT modify, this function by replacing "LastName" 
% with your last name.
function func = ControllerSun
% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #3: Modify, but DO NOT RENAME, the function initControlSystem. It is
% called once before the simulation loop starts. If there are quantities 
% needed in your control function that only need to be defined once, you 
% can define them here and store them in the structure "data" which 
% is supplied as an input to the function runControlSystem below. For 
% example, to store the variable a = 4 in the structure data, you can write 
% data.a = 4; This function should output your data file and the initial
% values of the control input to apply at the start of the simulation
function [actuators,data] = initControlSystem(sensors,references,parameters,data)

% load
M_sym = parameters.symEOM.M;
C_sym = parameters.symEOM.C;
N_sym = parameters.symEOM.N;
tau_vector = parameters.symEOM.tau;

% linearlization
syms q1 q2 v1 v2 tau1 real
q_dot_sym = [v1; v2];
invM = inv(M_sym); 
q_ddot_sym = invM * (tau_vector - C_sym*q_dot_sym - N_sym);
f_sym = [q_dot_sym; q_ddot_sym];
x_sym = [q1; q2; v1; v2];
u_sym = tau1;

% equilibrium
q_eq = [0; 0]; 
v_eq = [0; 0];
N_eq = double(subs(N_sym, {q1, q2, v1, v2}, {0, 0, 0, 0}));
u_eq = N_eq(1); 
    
data.x_eq = [q_eq; v_eq];
data.u_eq = u_eq;

% linearize
vars_sub = [q1, q2, v1, v2, tau1];
vals_sub = [data.x_eq', u_eq];
A = double(subs(jacobian(f_sym, x_sym), vars_sub, vals_sub));
B = double(subs(jacobian(f_sym, u_sym), vars_sub, vals_sub));

% define poles
poles = [-3, -4, -1, -20];

% calculate K
data.K = place(A, B, poles);

C_ref = [0 1 0 0];
data.k_ref = -inv(C_ref * inv(A - B*data.K) * B);

% integral gain
data.k_int = -0.4; 
% data.k_int = 0; 

data.int_error = 0; 

% define the torque to apply in the nonlinear system
actuators.tau1 = 0;

end

%
% STEP #4: Modify, but DO NOT RENAME, the function runControlSystem. It is 
% called every time through the simulation loop. Within this function, you 
% will define the control input that is applied over time. The inputs to 
% this function are sensors, reference, parameters, and data.  
% Data contains whatever variables you defined in initControlSystem.
% (You can also store things in data within runControlSystem for later 
% access). References contains a user defined reference signal.  
% Sensors and parameters contain the variables that describe the system.
% See the accompanying README file for more details.
function [actuators,data] = runControlSystem(sensors,references,parameters,data)

% setup
x = [sensors.q1; sensors.q2; sensors.v1; sensors.v2];
r = references.q2;

% state deviation
x_dev = x - data.x_eq;

% calculate error
current_error = x(2) - r;

u_test = -data.K * x_dev + data.k_ref * r - data.k_int * data.int_error;
    
limit = parameters.tauMax;

% check saturation
is_saturated = abs(u_test) > limit;
    
% only integrate if not saturated
if ~is_saturated
   data.int_error = data.int_error + (current_error * parameters.tStep);
end

% control Law
% u = -Kx + k_ref*r - k_int * integral_error
u_calc = -data.K * x_dev + data.k_ref * r - data.k_int * data.int_error + data.u_eq;
    
% apply limits
actuators.tau1 = max(min(u_calc, limit), -limit);

end



