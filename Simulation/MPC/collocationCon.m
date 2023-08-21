% Direct collocation as nonlincon to enforce dynamic constraints
function [c, ceq, dc, dceq] = collocationCon(dv,p,c)
    
%%%% INPUT
    % dv: decision variable vector
    % c: control parameter struct
    % p: physical parameter struct

    %%%% OUTPUT
    % ceq: 
    % collocation constraint, 4(N-1) x 1  => midpoint of cubic spline
    % must match equations of motion, only accurate over small intervals
    %     
    % dceq: jacobian of ceq wrt decision vars; 4(N-1) x 5N

    % control parameters
    dt = c.dt_p;    % prediction time interval
    N = c.N;        % prediction horizon
    step = 1e-8;    % step size for approximating jacobian with finite differencing

    % initialize collocation constraint vector
    ceq = zeros(4*(N-1), 1);
    dceq = zeros(4*(N-1), 5*N);

    %%%% compute collocation constraint feasibility (part of Lagrangian with sqp) at each timestep
    for i=1:(N-1)
        % extarct from dv states and control at each interval
        z1 = dv(4*i-3:4*i);
        z2 = dv(4*i+1:4*(i+1));
        u1 = dv(4*N+i);
        u2 = dv(4*N+i+1);
        
        % feasibility for matching spline and dynamics
        feas = collocationPoints(z1,z2,u1,u2,dt,p);
        ceq(4*i-3:4*i) = feas;

        % jacobian wrt states
        for j = 1:4
          % small step in direction of jth state at time step i
          dz = zeros(4,1);
          dz(j) = step;

          % corresponding change in constraint feasibility
          dceq1 = collocationPoints(z1+dz, z2, u1, u2, dt, p) - feas;
          dceq2 = collocationPoints(z1, z2+dz, u1, u2, dt, p) - feas;

          % finite difference approximation of the gradient dceq/dz
          dceq(4*i-3:4*i, 4*(i-1)+j) = dceq1/step;
          dceq(4*i-3:4*i, 4*i+j) = dceq2/step;
        end
        
        % jacobian wrt control
        % small step in direction of u at time step i
        du = step;

        % corresponding change in constraint feasibility
        dceq1 = collocationPoints(z1, z2, u1+du, u2, dt, p) - feas;
        dceq2 = collocationPoints(z1, z2, u1, u2+du, dt, p) - feas;

        % finite difference approximation of the gradient dceq/du
        dceq(4*i-3:4*i, 4*N+i) = dceq1/step;
        dceq(4*i-3:4*i, 4*N+i+1) = dceq2/step;

    end
   
    c = [];
    dc = [];
end

% Computes feasibility of above constraint at some specified interval
function ceq = collocationPoints(z1,z2,u1,u2,dt,p) 
    % calculate z' = f(z,u)
    dz1 = eom(z1, u1, p);
    dz2 = eom(z2, u2, p);

    % calculate mid points of break points 
    % based on cubic spline formula (refer to underactuated robotics Ch.10)
    zc = 0.5*(z1+z2) - (dt/8)*(dz2-dz1);
    dzc = (3/(2*dt))*(z2-z1) - (1/4)*(dz1+dz2);
    
    % dynamics at collocation point must fit into cubic spline
    % with linear interpolation between successive control inputs
    ceq = eom(zc, (u1+u2)/2, p) - dzc;
end