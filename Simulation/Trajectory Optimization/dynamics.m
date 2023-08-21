function dx = dynamics(t, x, u, p, c)
    
    %%%% INPUT:
    % t: time step
    % x: state vector [x, theta, dx, dtheta]
    % u: system input
    % p: physical parameters
    % c: simulation and system constants
    
    %%%% OUTPUT:
    % z: decision variable (column) vector containing the x_i and u_i at each timestep
    % N*(nx+nu) by 1

    %%%% unpack constants
    x_f = c.x_f;
    K = c.K;
    S = c.S;
    sim_time_dc = c.dt;
    
    %%%% switch between LQR and D.C. based on theta
    
    % cost-to-go from algebraic Ricatti eqn as a metric of closeness
    if (x-x_f)'*S*(x-x_f) < 1.0
        u = - K*(x-x_f);
    
    % interpolate from D.C. if far away from equilibrium
    else
        % find proper i to sample from
        i = ceil(t/sim_time_dc);

        % trim off ends
        if i == 0
            i = 1;
        end
        if i >= numel(u)
            i = numel(u) - 1;
        end

        dt = mod(t,sim_time_dc)/sim_time_dc;
        
        % construct linear spline for u
        u = u(i)*(1-dt) + u(i+1)*dt;
    end
    
    %%%% dynamics
    dx = eom(x, u, p);
end
