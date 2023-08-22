% Initial States
p.z0 = [0.0;          % cart position
      (pi/180)*0;   % pendulum angle (wrt gravity)
      0.0;          % cart velocity
      0.0];         % pendulum angular velocity

% Reference
p.zt = @(t) [0; pi; 0; 0]; 

%% 
for i = 1:9
    load("trial" + string(i) + ".mat")
    r
    % r.t_s = linspace(0,r.tspan,r.tspan*r.fps);
    % r = rmfield(r, 'record');
    % save("trial" + string(i) + ".mat")
end