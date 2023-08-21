clc; clear; clear global; close all;
% 1 2 5 6 7 8 9 10 11 12 13

ctrl = '0';
trial = '12';

record = 1;

filename = "trial" + ctrl + '_' + trial; 

%% Parameters

%%%% physical and simulation parameters
load("C:\Users\hansb\OneDrive\Documents\School\Mo&T\CartPole\Sim\MPC Direct Collocation\params\trial"+trial+".mat",'p','r')
r.record = record;
r.filename = "animation\" + filename + ".gif";
if record && exist("animation\" + filename, "file") 
    delete(filename);   % reset existing file
end

try
    load("C:\Users\hansb\OneDrive\Documents\School\Mo&T\CartPole\Sim\MPC Direct Collocation\params\trial"+trial+".mat",'p_un')
    p_un;
catch
    p_un = p;
end

%%
% only modify time span if necessary
r.tspan = 15;   flag = 1;
if exist('flag','var')
    t_s = linspace(0,r.tspan,r.tspan*r.fps); r.t_s = t_s;
else
    try 
        t_s = r.t_s;
    catch
        t_s = linspace(0,r.tspan,r.tspan*r.fps); r.t_s = t_s;
    end
end

%%%% controller parameters
try
    load("localparams\" + filename + ".mat", 'c')
    disp("Controller parameters loaded")
catch
    load("C:\Users\hansb\OneDrive\Documents\School\Mo&T\CartPole\Sim\MPC Direct Collocation\params\trial" + trial + ".mat",'c')
    % manual set LQR and swing up gains
    c.Q = diag([2 10 4 1]);   % state error 
    c.R = 6;        % actuator effort 
    Ke1 = 0.87;
    Ke2 = 0.7;
    c.Ke = str2double(ctrl)*Ke1 + (~str2double(ctrl))*Ke2;  % energy error gain
    [c.K,c.S] = LQR(p,c);
    c.dt_p = 0.05; % control loop interval
    disp("Controller parameters updated")
end
try
    c.T;
    c = rmfield(c,{'T', 'N', 't_pred'});
end

% update zt from old files
try 
    c.zt(0.1);
catch
    c.zt = @(t) [0;pi;0;0];   
end

% modify initial state only if necessary
% c.z0(1) = 0;          
% if ctrl == "1" && c.z0(2) == 0
%     c.z0(2) = 0.0017;
% end

u = @(t,z)control(t,z,p,c,str2double(ctrl));

%% Simulation
%%%% Function Handle
dynFun = @(t,z)cartPoleDynamics(t,z,c,u,p_un);

% global variable to keep track of fictional control loop time and input during iterated call of ode45
global t_prev u_prev z_prev
t_prev = -inf;
u_prev = 0;
z_prev = 0;

%%%% Simulate the system!
disp("Simulation in progress!!")
options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~, z] = ode45(dynFun, t_s, [c.z0;0], options);   %  <-- This is the key line!
u = z(:,5);
z = z(:,1:4)';

%% Plot 
if record
    figure(1); clf; hold on;
    plotCartPole(t_s,z,u);  % Moved plotting to its own function
    set(gcf,"WindowState",'maximized')
    saveas(gcf,"plots\" + filename + ".jpg")
end

%% Animation
% Create and clear a figure
figure(2); clf;
set(gcf,"position", [0,0,900,600])  % set screen size
movegui(gcf, 'center'); % center animation
animate(z,p,r)  % <-- produce animation

disp("Done!!")
close all

%% Save parameters
if record
    save("localparams\" + filename +  ".mat", 'c', 'p', 'r')
    save("results\" + filename + ".mat", 'z', 'u')
end