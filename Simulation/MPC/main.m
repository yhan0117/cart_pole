clc; clear; clear global; close all;
% TODO
% robustness -> change first initial guess, multistart, prove recursive robustness 
% speed      -> analytical hessian, orthogonal collocation
% reference  -> track pendulum position => change cost function
% test       -> swing up, tracking with normally distributedmuncertainty 

%% Constants
%%%% Initial States
trial = 1;
z0 = [0.0;          % cart position
      (pi/180)*179.9;   % pendulum angle (wrt gravity)
      0.0;          % cart velocity
      0.0];         % pendulum angular velocity
c.z0 = z0;

%%%% Physical Parameters 
p.m1 = 1.0; % (kg) Cart mass (carriage + encoder + printed mount)
p.m2 = 0.2; % (kg) pole mass
p.g = 9.81; % (m/s^2) gravity
p.l = 0.5;  % (m) pendulum (pole) length
p.L = 1.2;  % (m) rail length
p.s = 350*0.05; % (N) max input force (max torque * gear radius)

%% Control Parameters

%%%% Reference
% zt = @(t) [0; pi; 0; 0]; c.zt = zt; % static reference => upward equilibrium
% zt = @(t) [0.8*round(0.5*(1+cos(2*pi*t/5)))-0.4; pi; 0; 0]; c.zt = zt; % square wave
zt = @(t) [0.4*sin(2*pi*t/2); pi; 0; 0]; c.zt = zt; % sin wave
% zt = @(t) [0; pi; 0; 0; 0.4*sin(2*pi*t/2); p.L]; c.zt = zt;    % include extra variable (x,y)
c.mode = 1; % mode 1 tracks state variables, mode 0 tracks pendulum translational position

%%%% MPC optimization params
% 1 step control horizon
N = 60;  c.N = N;       % predicition horizon 
t_pred = 1.5; c.t_pred = t_pred; % prediction time span
c.dt_p = t_pred/N;      % prediction step size

% external disturbance @ t = 3sec
c.t_d = 0.15;  % impact time duration
c.ui = 0;    % impact magnitude

% uncertainty
c.std = 0.0;

% options for NLP Solver
options = optimoptions('fmincon','TolX', 1e-9, 'TolFun', 1e-9, 'TolCon', 1e-9, 'MaxIterations', 100,'MaxFunEvals',10000,...
    'DiffMinChange',1e-5, 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'Display', 'final', 'Algorithm','sqp', 'UseParallel', true);


%%%% Cost weighing factor
% trajectory tracking
% c.T = 1+(1.2*(0:N-1)/N).^16;        % weight for each break point
% c.Q = diag([eps 100.0 eps eps]);
% c.R = 0.06;                          % actuation effort

% swing up
% c.T = repmat(1.3,1,N);
% c.T(end) = 110;
% c.Q = diag([1 1 1 1]);
% c.R = 0.08;

%%%% feedback controller u = f(t,z)
u = @(t,z)control(t,z,p,c,options);

%% Simulation Parameters
record = 1; r.record = record;
filename = "animation\trial"+trial+".gif"; r.filename = filename;

fps = 30; r.fps = fps;       % simulation resolution
tspan = 15; r.tspan = tspan; % duration   
t_s = linspace(0,tspan,fps*tspan);   r.t_s = t_s;   % simulation time stamps

%% Simultation

%%%% Add uncertainty
p_un = p;
p_un.m1 = p.m1*(1+randn*c.std);
p_un.m2 = p.m2*(1+randn*c.std);
p_un.l = p.l*(1+randn*c.std);

%%%% Function Handle
dynFun = @(t,z)dynamics(t,z,c,u,p_un);

% global variable to keep track of fictional control loop time and input during iterated call of ode45
global dv_prev t_prev output;  
t_prev = -inf;
output = [];

% this is also the first initial guess of the optimization solver
% --> offline opt first to improve speed => load("OptTrajWConstraint.mat")
dv_prev = zeros(5*N,1);
zt0 = zt(0);
for i=1:N
    dv_prev(4*i-3:4*i) = z0 + (i-1)*(zt0(1:4)-z0)/(N-1);  % linear interpolation 
end

%%%% Simulate the system!
disp("Simulation in progress!!!")
options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~, z] = ode45(dynFun, t_s, [z0;0], options);   %  <-- This is the key line!
u = z(:,5);
z = z(:,1:4)';

%% Plot 
if record
    figure(1); clf; hold on;
    plotCartPole(t_s,z,u);  % Moved plotting to its own function
    set(gcf,"WindowState",'maximized')
    saveas(gcf,"plots/trial"+trial+".jpg")
end

%% Save parameters
if record
    save("params\trial" + trial + ".mat", 'c', 'options', 'p', 'r','p_un')
    save("results\trial" + trial + ".mat", 'z', 'u', 'output')
end

%% Animation
if record && exist(filename, "file") 
    delete(filename)    % clear existing file
end

% Create and clear a figure
figure(2); clf;
set(gcf,"position", [0,0,900,600]) 
movegui(gcf, 'center'); % center animation
animate(z,p,r)  % <-- produce animation

disp("Done!!")
close all



