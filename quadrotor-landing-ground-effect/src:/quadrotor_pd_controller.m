%
% STEP #1: Rename this file by replacing "LastName" with your last name.
%

%
% STEP #2: Rename, but do NOT modify, this function by replacing "LastName" 
% with your last name.
function func = ControllerSun
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

% Save parameters
data.m  = parameters.m;
data.g  = parameters.g;
data.c1 = parameters.c1;
data.c2 = parameters.c2;

% Target small hover height
data.z0 = 0.001;   % 1 mm above ground

% Compute equilibrium thrust T0 at z0
FG0 = data.c1 / (data.z0^2 + data.c2);
data.T0 = data.m * data.g / (1 + FG0);

% Feedback gains
data.k1 = 30;    % position gain
data.k2 = 8.995;    % velocity gain

actuators.thrust = 0;

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

% Current states
z    = sensors.z;
zdot = sensors.zdot;

% Deviation from target
x1 = z - data.z0;   % position error
x2 = zdot;          % velocity

% Feedback law
u = -data.k1*x1 - data.k2*x2;

% Control input = equilibrium thrust + feedback correction
T = data.T0 + u;

% Safety clamp (must stay within physical thrust range)
Tmax = parameters.maxthrust;
Tmin = 0;
T = min(max(T, Tmin), Tmax);

% Send thrust command
actuators.thrust = T;

end














