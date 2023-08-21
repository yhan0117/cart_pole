% computes the gradient of constraints asssociated with system dynamics
function [h_i,dH_i] = dyConGradient(x_1, u_1, x_2, u_2, dt, p)

    %%%% INPUT
    % x_1: state at the start of the interval; nx by 1 
    % u_1: input at the start of the interval; nu by 1 
    % x_2: state at the end of the interval; nx by 1 
    % u_2: input at the end of the interval; nu by 1 
    % dt: duration of the interval; scalar
    
    %%%% OUTPUT
    % h_i: constraint value from dynamics_constraint; nx by 1
    % dH_i: jacobian of h_i w.r.t. [x_1, u_1, x_2, u_2]; nx by (2nx + 2nu)

    % integrate using ? order Runge-Kutta
    h_i = control_points(x_1, u_1, x_2, u_2, dt, p);
    
    %%%% numerical derivatives to compute dH
    % dH = [dh/dx1 dh/du1 dh/dx2 dh/du2]
    % where the partial derivatives are written (dh/dx1)_ij = dh_i/dx1_j
    
    % init
    delta = 1e-8;
    dH_i = zeros(numel(x_1), 2*(numel(x_1)+numel(u_1)));
    
    % x(1-4)
    for j=1:numel(x_1)
      dx = zeros(numel(x_1),1);
      dx(j) = delta;
      dHx_i_j = control_points(x_1 + dx, u_1, x_2, u_2, dt, p) - h_i;
      dHx_ip1_j = control_points(x_1, u_1, x_2 + dx, u_2, dt, p) - h_i;
      dH_i(:,j) = dHx_i_j/delta;
      dH_i(:,j + numel(x_1) + numel(u_1)) = dHx_ip1_j/delta;
    end
    
    % u
    for j=1:numel(u_1)
      du = zeros(numel(u_1),1);
      du(j) = delta;
      dHu_i_j = control_points(x_1, u_1 + du, x_2, u_2, dt, p) - h_i;
      dHu_ip1_j = control_points(x_1, u_1, x_2, u_2 + du, dt, p) - h_i;
      dH_i(:,j + numel(x_1)) = dHu_i_j/delta;
      dH_i(:,j + numel(x_1) + numel(u_1) + numel(x_2)) = dHu_ip1_j/delta;
    end
    
end

function h_i = control_points(x_1, u_1, x_2, u_2, dt, p)

    % calculate dz
    dx_1 = eom(x_1, u_1, p);
    dx_2 = eom(x_2, u_2, p);

    % calculate collocation points -> mid sample point
    Si_mid = 0.5*(x_1+x_2) - (dt/8)*(dx_2-dx_1);
    Si_mid_dot = (3/(2*dt))*(x_2-x_1) - 0.25*(dx_1+dx_2);
    
    % dynamics at collocation point must fit into cubic spline
    % --> constraint on system
    h_i = eom(Si_mid, (u_1+u_2)/2, p) - Si_mid_dot;
end