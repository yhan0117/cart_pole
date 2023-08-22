% Cart pole swing up with open loop trajectory optimization
clear;clear global;clc;close all

trial = 1;
record = 0; 
path = pwd;

%% Test Case Parameters
load("../Test Cases/trial" + trial + ".mat")

%% Controls <-- only section to modify amongst different control methods
% Discretization Constants
c.N = 100;      % number of sample points
c.dt = 5/N;     % sample time step size

% LQR
c.Q = diag([0.06,10,1,2]);
c.R = 1;
[c.K, c.S] = LQR(p,c);

% options for NLP Solver
c.options = optimoptions('fmincon','TolX', 1e-9, 'TolFun', 1e-9, 'TolCon', 1e-9, 'MaxIterations', 500,'MaxFunEvals',50000,...
    'DiffMinChange',1e-5, 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'Display', 'iter-detailed', 'Algorithm','interior-point', 'UseParallel', true);

% Offline calculation of swing up trajectory
disp("Calculating open loop swing up trajectory")
c.u_o = swingUp(p,c);

% feedback controller u = f(t,z)
u = @(t,z)control(t,z,p,c);


%% Simulation
disp("Producing simulation")
options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~, z] = ode45(@(t,z)cartPoleDynamics(t,z,u,p), r.t_s, p.z0, options);
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
