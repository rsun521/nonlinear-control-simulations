function Project1(controller,varargin)
% DesignDemo01   run simulation of quadrotor moving on vertical axis
%
%   DesignDemo01('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignDemo01('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatalog' : a cell array (e.g., {'y','xhat'}) with
%                                 the names of fields in controller.data -
%                                 if 'datafile' is defined (so data is
%                                 logged), then values in these fields will
%                                 also be logged and saved
%
%           'perturbmodel' : a flag - true or false (default) - that, if
%                            true, causes the true mass to be different
%                            from what we think it is
%           'display' : a flag - true (default) or false that, if false,
%                       will not show any graphics and will run the
%                       simulation as fast as possible (not in real-time)
%
%   Regardless of how the function is called, it will clear the current
%   figure and will show the simulation. To quit, type 'q' when this figure
%   is in the foreground.

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'perturbmodel',false,@logical);
addParameter(p,'display',true,@islogical);
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Stop time.
process.tStop = 10;
% - Time step.
process.tStep = 1/50;
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','z','zdot'};

% Constants related to physical properties.
% - Acceleration of gravity.
process.g = 9.81;
% - Ground effect constants
process.c1 = 0.00830769;
process.c2 = 0.00615385;
% - Mass
process.m = 1.0;
if (process.perturbmodel)
    process.mTrue = process.m+(0.5*(-1+2*rand));
else
    process.mTrue = process.m;
end
% - Maximum thrust
process.maxthrust = 20;

% DEFINE VARIABLES

% Time
process.t = 0;
% Z-Position
process.z = 1+rand;
% Z-Velocity
process.zdot = 8*(rand-.5);
% Collision checker
process.result = 0;

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','g','m','maxthrust','c1','c2'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% References
controller.references = struct('z',0);
% Sensors
controller.sensors = GetSensors(process);
% Actuators
controller.running = true;
tic
try
    [controller.actuators,controller.data] = ...
        controller.init(controller.sensors, ...
                           controller.references, ...
                           controller.parameters, ...
                           controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'did not return a structure ''actuators'' with the right\n' ...
             'format. Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name);
    controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tComputation = toc;

end


function sensors = GetSensors(process)
sensors.t = process.t;
sensors.z = process.z;
sensors.zdot = process.zdot;
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.z; process.zdot];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.thrust];

% Bound input
if u<0
    u = 0;
elseif u>process.maxthrust
    u = process.maxthrust;
end

% Add disturbance
%   (nothing)
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.z = x(1,1);
process.zdot = x(2,1);
if process.z<0
    process.result = 1;
end
end

function xdot = GetXDot(t,x,u,process)
% unpack x and u
z = x(1,1);
zdot = x(2,1);
thrust = u(1,1);
% compute rates of change
d_z = zdot;
d_zdot = (1+process.c1/(z^2+process.c2))*(thrust/process.mTrue)-process.g;
% pack xdot
xdot = [d_z; d_zdot];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'thrust'}))&&(length(fieldnames(actuators))==1)
    if isnumeric(actuators.thrust)
        if isscalar(actuators.thrust)
            iscorrect = true;
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('thrust',0);
end

function res = ShouldStop(process)
% Check stopping condition (crash, win)
if process.result==1
    res = true;
else
    res = false;
end
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE
    
    % Clear the current figure.
    clf;
    
    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;

    
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('CONTROLLER: %s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.05,0.12,...
        sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');
    fig.text.result=text(0.05,0.06,...
        sprintf('above the floor'),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');
    
    
    fig.x.axis = axes('position',[0.55,0.6,0.4,0.35],'fontsize',fs);
    axis([0,process.tStop,-3,3]);
    hold on;
    fig.x.z = plot(nan,nan,'linewidth',2);
    fig.x.zdot = plot(nan,nan,'linewidth',2);
    fig.x.floor = plot([0 process.tStop],[0 0],...
                      'linewidth',1,'linestyle','--','color','k');
    fig.x.legend = legend({'position','velocity'});
    xlabel('time');
    
    
    fig.u.axis = axes('position',[0.55,0.1,0.4,0.35],'fontsize',fs);
    delta = 0.1*process.maxthrust;
    axis([0,process.tStop,0-delta,process.maxthrust+delta]);
    hold on;
    fig.u.u = plot(nan,nan,'linewidth',3,'color',[0.9290    0.6940    0.1250]);
    fig.u.umin = plot([0 process.tStop],[0 0],...
                      'linewidth',1,'linestyle','--','color','k');
    fig.u.umax = plot([0 process.tStop],process.maxthrust*[1 1],...
                      'linewidth',1,'linestyle','--','color','k');
    fig.u.legend = legend('thrust');
    xlabel('time');       
    
    % Create an axis for the view from frame 0.
%     fig.view0.axis = axes('position',[0 0 1 1]);
    fig.view0.axis = axes('position',[0 0.02 0.5 1]);
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis equal;
%     axis(1.0*[-1 1 -1 1 -1 1]);
    axis(1.25*[-1.5 1.5 -1.5 1.5 -3 3]);
    axis manual;
    hold on;
    axis off;
%     view([37.5,20]);
    view([100,15]);
    box on;
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[0;-2;2],'style','local');
    
    pFrame = [0 1 0 0;
              0 0 1 0;
              0 0 0 1];
    fig.geom.pFrame0_in0 = pFrame;
    fig.geom.pFrame1_in1 = pFrame;
    
    [fig.geom.pRobot_in1,fig.geom.fRobot]=GetRobotModel('quadmodel.mat');
    
    o_1in0 = [0;0;process.z];
    R_1in0 = eye(3);
    fig.geom.pFrame1_in0 = Transform(o_1in0,R_1in0,fig.geom.pFrame1_in1);
    fig.geom.pRobot_in0 = Transform(o_1in0,R_1in0,fig.geom.pRobot_in1);
    
    fig.view0.frame0 = DrawFrame([],fig.geom.pFrame0_in0);
    fig.view0.frame1 = DrawFrame([],fig.geom.pFrame1_in0);
    fig.view0.robot = DrawMesh([],fig.geom.pRobot_in0,fig.geom.fRobot,'y',0.9);
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
else
    % UPDATE FIGURE
    
    set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop));
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);
    
    t = get(fig.x.z,'xdata');
    x = get(fig.x.z,'ydata');
    set(fig.x.z,'xdata',[t process.t],'ydata',[x process.z]);
    
    t = get(fig.x.zdot,'xdata');
    x = get(fig.x.zdot,'ydata');
    set(fig.x.zdot,'xdata',[t process.t],'ydata',[x process.zdot]);
    
    t = get(fig.u.u,'xdata');
    x = get(fig.u.u,'ydata');
    thrust_const = max([min([controller.actuators.thrust process.maxthrust]) 0]);
    set(fig.u.u,'xdata',[t process.t],'ydata',[x thrust_const]);
    
    
%     set(fig.x.q3,'xdata',data.t,'ydata',data.o_1in0(3,:));
%     set(fig.x.v3,'xdata',data.t,'ydata',data.v_01in0(3,:));
%     set(fig.x.q3d,'xdata',data.t,'ydata',data.xd(1,:));
%     set(fig.x.v3d,'xdata',data.t,'ydata',data.xd(2,:));
%     
%     set(fig.u.u,'xdata',data.t,'ydata',data.ud);
    
    o_1in0 = [0;0;process.z];
    R_1in0 = eye(3);
    fig.geom.pFrame1_in0 = Transform(o_1in0,R_1in0,fig.geom.pFrame1_in1);
    fig.geom.pRobot_in0 = Transform(o_1in0,R_1in0,fig.geom.pRobot_in1);
    
    fig.view0.frame1 = DrawFrame(fig.view0.frame1,fig.geom.pFrame1_in0);
    fig.view0.robot = DrawMesh(fig.view0.robot,fig.geom.pRobot_in0);

    if process.result==1
        set(fig.text.result,'string','CRASHED!','color','red');
    end
    
end
drawnow;
end

function [p,f]=GetRobotModel(filename)
load(filename);
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)
    
    % Update figure (create one if fig is empty).
    if (process.display)
        fig = UpdateFigure(process,controller,fig);
    end
    
    % Update data.
    if (~isempty(process.datafile) && controller.running)
        [process,controller] = UpdateDatalog(process,controller);
    end
    
    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if ((process.t>=process.tStop)||done||ShouldStop(process))
        break;
    end
    
    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);
    
    % Wait if necessary, to stay real-time.
    if (process.display)
        while (toc(tStart)<process.t-process.tStart)
            % Do nothing
        end
    end
    
end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tComputation',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
process.log.controller.tComputation(:,process.log.count) = ...
    controller.tComputation;
names = fieldnames(controller.sensors);
for i=1:length(names)
    name = names{i};
    process.log.controller.sensors.(name)(:,process.log.count) = ...
        controller.sensors.(name);
end
names = fieldnames(controller.actuators);
for i=1:length(names)
    name = names{i};
    process.log.controller.actuators.(name)(:,process.log.count) = ...
        controller.actuators.(name);
end
for i=1:length(process.controllerdatatolog)
    name = process.controllerdatatolog{i};
    try
        process.log.controller.data.(name)(:,process.log.count) = ...
            controller.data.(name);
    catch exception
        warning(['Saving element ''%s'' of data for controller\n',...
                 '     ''%s''',...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],...
                 name,controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
        return
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);

% Get sensor values
controller.sensors = GetSensors(process);

% Get actuator values (run controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tComputation = toc;
else
    controller.tComputation = 0;
end

end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end
 
function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end
 
function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end
 
function R = R_ZYX(theta)
R = RZ(theta(1))*RY(theta(2))*RX(theta(3));
end

function thetadot = GetAngularRates_ZYX(theta,w)
c2 = cos(theta(2));
s2 = sin(theta(2));
c3 = cos(theta(3));
s3 = sin(theta(3));
A = [   -s2     0       1;
        c2*s3   c3      0;
        c2*c3   -s3     0];
thetadot = A\w;
end

function wHat = wedge(w)
wHat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end

function p_inj = Transform(o_kinj,R_kinj,p_ink)
p_inj = zeros(size(p_ink));
for i=1:size(p_ink,2)
    p_inj(:,i) = o_kinj + R_kinj*p_ink(:,i);
end
end

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function mesh = DrawMesh(mesh,p,f,color,alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end

function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);

    % Plot floor
    fill3(4*[-1 -1 1 1 -1],4*[-1 1 1 -1 -1],[0 0 0 0 0],[.5 .5 .5],'facealpha',.5)
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


