% Cart pole swing up and stabilization with model predicitve control
% TODO
%{ 
    robustness -> change first initial guess, multistart, prove recursive robustness 
    speed      -> analytical hessian, orthogonal collocation
    reference  -> track pendulum position => change cost function
    test       -> swing up, tracking with normally distributed uncertainty 
%}
clc; clear; clear global; close all;

trial = 1;
record = 0; 
path = pwd;

%% Test Case Parameters
load("../Test Cases/trial" + trial + ".mat")

%% Controls <-- only section to modify amongst different control methods
% tracking pendulum translational position
% zt = @(t) [0; pi; 0; 0; 0.4*sin(2*pi*t/2); p.L]; p.zt = zt;    % include extra variable (x,y)
% c.mode = 1;

% MPC optimization params
% 1 step control horizon
N = 60;  c.N = N;       % predicition horizon 
t_pred = 1.5; c.t_pred = t_pred; % prediction time span
c.dt_p = t_pred/N;      % prediction step size

% options for NLP Solver
options = optimoptions('fmincon','TolX', 1e-9, 'TolFun', 1e-9, 'TolCon', 1e-9, 'MaxIterations', 100,'MaxFunEvals',10000,...
    'DiffMinChange',1e-5, 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'Display', 'final', 'Algorithm','sqp', 'UseParallel', true);

% Cost matrices
% trajectory tracking
c.T = 1+(1.2*(0:N-1)/N).^16;      % weight for each break point
c.Q = diag([eps 100.0 eps eps]);  % error cost
c.R = 0.06;                       % actuation effort

% swing up
% c.T = repmat(1.3,1,N);
% c.T(end) = 110;
% c.Q = diag([1 1 1 1]);
% c.R = 0.08;

% global variable to keep track of simulated control loop time and input 
% during iterated call of ode45
global dv_prev t_prev output;  
t_prev = -inf;
output = [];
dv_prev = zeros(5*N,1); % <-- first initial guess of the optimization solver
zt0 = zt(0);
for i=1:N
    dv_prev(4*i-3:4*i) = p.z0 + (i-1)*(zt0(1:4)-p.z0)/(N-1);  % linear interpolation 
end

% feedback controller u = f(t,z)
u = @(t,z)control(t,z,p,c,options);

%% Simultation
disp("Producing simulation")
options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~, z] = ode45(@(t,z)cartPoleDynamics(t,z,c,u,p), r.t_s, p.z0, options);
u = z(:,5);
z = z(:,1:4)';

%% Save data
if record
    save("data\trial" + trial + ".mat", 'z', 'u', 'c')
end

%% Plot 
cd ../Visuals/
if record
    figure(1); clf; hold on;
    plotCartPole(t_s,z,u); % <-- plot results
    set(gcf,"WindowState",'maximized')
    saveas(gcf, path + "\plots\trial" + trial + ".jpg") 
end

%% Animation
filename = path + "\animations\trial" + trial + ".gif";
if record && exist(filename, "file") 
    delete(filename)    % clear existing file
end

% Create and clear a figure
figure(2); clf;
set(gcf,"position", [0,0,900,600])  % set window size
movegui(gcf, 'center');         % center animation
animate(z,p,r,record,filename)  % <-- produce animation

disp("Done!!")
close all


