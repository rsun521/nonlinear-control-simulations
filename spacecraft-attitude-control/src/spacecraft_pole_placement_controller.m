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

% Desired eigenvalues
data.des_poles = [-2.5, -3.0, -3.5];

% Store constants and desired equilibrium
J1 = parameters.J1; 
J2 = parameters.J2; 
J3 = parameters.J3;
we = references.r(:);                 % desired equilibrium angular velocity (3x1)

% ---- Build linearized model about we ----
% Rigid-body Euler equations in component form:
%   w1dot = ((J2-J3)/J1)*w2*w3 + (1/J1)*tau1
%   w2dot = ((J3-J1)/J2)*w3*w1 + (1/J2)*tau2
%   w3dot = ((J1-J2)/J3)*w1*w2          
c1 = (J2 - J3)/J1;
c2 = (J3 - J1)/J2;
c3 = (J1 - J2)/J3;

A = [0, c1*we(3), c1*we(2);
     c2*we(3), 0, c2*we(1);
     c3*we(2), c3*we(1), 0];

% Inputs are tau = [tau1; tau2; 0], so B = J^{-1} * [I2; 0]
B = [1/J1, 0;
     0, 1/J2;
     0, 0];

% ---- Place poles & save controller data ----
% Place returns K (2x3) for multi-input systems
K = place(A,B,data.des_poles);

% Store parameters
data.J1 = J1; 
data.J2 = J2; 
data.J3 = J3;
data.A  = A;  
data.B  = B;  
data.K  = K;
data.we = we;

% Initial torques
actuators.tau1 = 0;
actuators.tau2 = 0;
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

% Current angular velocity
w = [sensors.w1; sensors.w2; sensors.w3];

we_new = references.r(:);
if any(abs(we_new - data.we) > 1e-12)

    J1 = parameters.J1; 
    J2 = parameters.J2; 
    J3 = parameters.J3;
    c1 = (J2 - J3)/J1; 
    c2 = (J3 - J1)/J2; 
    c3 = (J1 - J2)/J3;

    A = [0, c1*we_new(3), c1*we_new(2);
         c2*we_new(3), 0, c2*we_new(1);
         c3*we_new(2), c3*we_new(1), 0];

    B = [1/J1, 0; 0, 1/J2; 0, 0];

    data.K  = place(A,B,data.des_poles);
    data.A  = A; data.B = B; data.we = we_new;
end

% State-feedback about equilibrium: tau = -K*(w - we)
e = w - data.we;
u = -data.K * e;

% Output torques
actuators.tau1 = u(1);
actuators.tau2 = u(2);
end