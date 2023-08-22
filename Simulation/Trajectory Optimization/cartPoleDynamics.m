function dx = cartPoleDynamics(t,z,u,p)
    
    %%%% INPUT:
    % t: time step
    % x: state vector [x, theta, dx, dtheta]
    % u: system input
    % p: physical parameters
    % c: simulation and system constants
    
    %%%% OUTPUT:
    % z: decision variable (column) vector containing the x_i and u_i at each timestep
    % N*(nx+nu) by 1
    
    %%%% switch between LQR and D.C. based on theta
    u = u(t,z);
    
    %%%% dynamics
    dz = eom(z, u, p);
end
