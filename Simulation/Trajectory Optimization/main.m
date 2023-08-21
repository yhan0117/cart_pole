% simulates a trajectory for the cartpole system
clear;clear global;clc;close all

%%%% Physical Parameters 
p.M = 1.0;      % (kg) cart mass (carriage + encoder + printed mount)
p.m = 0.3;                 % (kg) pole mass
p.g = 9.81;                % (m/s^2) gravity
p.l = 0.5;                 % (m) pendulum (pole) length
p.L = 1.5;                 % (m) rail length
p.s = 500*0.04;            % (N) motor saturation

%%%% Simulation Specs
recording = 0;
filename = "Animation.gif";
if recording
    % clear file
    if exist(filename, "file") 
        delete(filename)
    end
end

fps = 50;                   % simulation resolution
tspan = 5;                 % duration   
t_ = linspace(0,tspan,fps*tspan);   % simulation time stamps

%%%% D.C. Constants
N = 20;                    % number of samples/ control points
dt = 5/N;                   % constraint and objective step size
nx = 4;                     % number of states
nu = 1;                     % number of control inputs
% t_ = linspace(0,tspan,N);   % simulation time stamps

%%%% initial and final conditions
% x: state vector (x, theta, dx, dtheta) 
% u: input trajectory
x_0 = [0.0; (pi/180)*0 ; 0; 0];
x_f = [0; pi; 0; 0];


%%%% LQR for upward equilibrium
[K, S] = LQR(p)
%%

%%%% Parameters to pass
c.N = N;
c.dt = dt;
c.nx = nx;
c.nu = nu;
c.x_f = x_f;
c.K = K;
c.S = S;

% initial guess of optimization
load("OptTrajWConstraint50.mat")
c.z0 = z; 

%%%% Swing up trajectory through direct collocation
tic
disp("Calculating swing up trajectory")
z = swingUp(x_0,p,c);
toc

u = zeros(nu,0);
z_ = zeros(nx,0);

% extract each timestep from z vector
for i=1:N
    [z_i, u_i] = z_index(i, nx, nu);
    u(:,i) = z(u_i);
    z_(:,i) = z(z_i);
end
%%
% save result as .mat for future use
% cd ..
% delete("OptTrajWConstraint.mat")
% save("OptTrajWConstraint50.mat", "u", "z_", "z")
% cd 'Trajectory Optimization'

%%%% SIMULATE!!!
% add uncertainty
% p.l = p.l*0.995;
% p.m = p.m*1.08;

disp("Producing simulation")
        options = odeset('MaxStep', 0.1,'RelTol',1e-8,'AbsTol',1e-8);
        odeFun = @(t,x) dynamics(t, x, u, p, c);
[~, z] = ode45(odeFun, t_, x_0, options);
z = z';


%%%% Plots
% figure(1); clf; hold on;
% plotCartPole(t_,z);  % Moved plotting to its own function
% figure(2); clf; hold on;
% plotMotor(t_,z);

%%
%%%% Animation
% Convert states to cartesian positions
clear global
disp("Animation in progress") 
pos = cartPolePosition(z,p);
x1 = pos(1,:);
y1 = pos(2,:);
x2 = pos(3,:);
y2 = pos(4,:);

% Plotting parameters:
p.w = 0.6*p.l;  % Width of the cart
p.h = 0.4*p.l;  % Height of the cart
p.r = 0.1*p.l;  % Radius of the pendulum bob

% Compute the extents of the drawing, keeping everything in view
padding = 0.2*p.l;  %Free space around edges
xLow = min(min(x1 - 0.5*p.w,  x2 - p.r)) - padding;
xUpp = max(max(x1 + 0.5*p.w,  x2 + p.r)) + padding;
% xLow = -1.0;
% xUpp = 1.0;
yLow = min(min(y1 - 0.5*p.h,  y2 - p.r)) - padding;
yUpp = max(max(y1 + 0.5*p.w,  y2 + p.r)) + padding;
extents = [xLow,xUpp,yLow,yUpp];

% Create and clear a figure:
figure(1); clf;
hold on;    %  <-- This is important!
set(gcf,'DoubleBuffer','on');   % Prevents flickering (maybe??)

if recording
    for k = 1:length(t_)
        % Redraw the image
        drawCartPole(t_(k),pos(:,k),extents,p);
        
        % Saving the figure
        frame = getframe(gcf);
    
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if k == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime', 1/fps);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', 1/fps);
        end
    end
else
    time = 0;
    tic;
    while time <= t_(end)
    
        % Compute the position of the system at the current real world time
        posDraw = interp1(t_',pos',time')';
        
        % Redraw the image
        drawCartPole(time,posDraw,extents,p);
    
        % Update current time
        time = toc;
    end
end


disp("done!")
close