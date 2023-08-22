% Cart pole swing up and stabilization with energy shaping + LQR control
clc; clear; clear global; close all;

trial = 1;
record = 0; 
ctrl = 1;   % controller type option: energy shaping (1) or resonance based (0)
path = pwd + "/" + string(ctrl);

%% Test Case Parameters
load("../Test Cases/trial" + trial + ".mat")

%% Controls <-- only section to modify amongst different control methods
% LQR gains
c.Q = diag([2 10 4 1]);   % state error 
c.R = 6;        % actuator effort 
[c.K,c.S] = LQR(p,c);

% swing up 
Ke1 = 0.87;
Ke2 = 0.7;
c.Ke = ctrl*Ke1 + (~ctrl)*Ke2;  % energy error gain
    
% control loop interval
c.dt_p = 0.05; 

% global variable to keep track of simulated control loop time and input 
% during iterated call of ode45
global t_prev u_prev acc
t_prev = -inf;
u_prev = 0;
acc = 0;

% feedback controller u = f(t,z)
u = @(t,z)control(t,z,p,c,ctrl);

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


