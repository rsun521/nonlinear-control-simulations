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
% called once before the simulation loop starts. 
function [actuators,data] = initControlSystem(sensors,references,parameters,data)

% Define symbolic variables
syms phi phidot v w e_lateral e_heading real % symbolic state variables
syms tauR tauL real % symbolic control variables
z = [phi; phidot; v; w; e_lateral; e_heading]; % state vector
tau = [tauR; tauL]; % control vector
g = [phi; v; w; e_lateral; e_heading]; % output vector

% Select equilibrium point
ze = [0; 0; 6.1; 0; 0; 0]; % equilibrium state
taue = [0; 0]; % equilibrium control

v_road = ze(3); % desired velocity along road
w_road = 0; % ze(3)/sensor.r_road % desired turning rate along road
f_sys = parameters.symEOM.f; % EOMs for [phi'' v' w']
de_lateral = -v*sin(e_heading); % 
de_heading = w-(((v*cos(e_heading))/(v_road+w_road*e_lateral))*w_road);
f = [phidot; f_sys; e_lateral; e_heading];

% build 6*1 dynamics using derivatives
f = [phidot; f_sys; de_lateral; de_heading];

% Jacobian of f stats z and input tau
Az_sym = jacobian(f, z);
Bt_sym = jacobian(f, tau);

% Evaluate A B at the equilibrium (ze, taue)
A = double(subs(Az_sym, [z;tau], [ze; taue]));
B = double(subs(Bt_sym, [z;tau], [ze; taue]));

data.A = A;
data.B = B;
data.ze = double(ze);
data.ue = double(taue);

% Matrix C
C = [1 0 0 0 0 0;  % phi
     0 0 1 0 0 0;  % v
     0 0 0 1 0 0;  % w
     0 0 0 0 1 0;  % e_lateral
     0 0 0 0 0 1]; % e_heading

data.C = C;

% LQR
Q = diag([200, 100, 15, 5, 120, 120]);
R = diag([1, 1]);

K = lqr(A, B, Q, R);
data.K = K;

% Observer
poles_obs = -[8; 9; 10; 11; 12; 13];
L = place(A', C', poles_obs).';
data.L = L;

% Time step
data.dt = parameters.tStep;

% Initial measurement
y0 = [sensors.phi; sensors.v; sensors.w; sensors.e_lateral; sensors.e_heading];
z_hat0 = [sensors.phi; 0; sensors.v; sensors.w; sensors.e_lateral; sensors.e_heading];

data.z_hat  = z_hat0;
data.u_prev = data.ue;

% Initial control
actuators.tauR = data.ue(1);
actuators.tauL = data.ue(2);

end

%
% STEP #4: Modify, but DO NOT RENAME, the function runControlSystem. It is 
% called every time through the simulation loop.
function [actuators,data] = runControlSystem(sensors,references,parameters,data)

A = data.A;
B = data.B;
C = data.C;
K = data.K;
L = data.L;
ze = data.ze;
ue = data.ue;

dt = data.dt;
z_hat = data.z_hat;
u_prev = data.u_prev;

% Current measurement from sensors
y = [sensors.phi; sensors.v; sensors.w; sensors.e_lateral; sensors.e_heading];

X = y - C*z_hat;
z_hatdot = A*(z_hat - ze) + B*(u_prev - ue) + L*X;

z_hat = z_hat + dt*z_hatdot;

u = ue - K*(z_hat - ze);

tauMax = 100;           
u = max(min(u, tauMax), -tauMax);

data.z_hat = z_hat;
data.u_prev = u;

actuators.tauR = u(1);
actuators.tauL = u(2);




end