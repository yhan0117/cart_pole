% This function represents the control law that computes the input
% based on full state feedback
function u = control(t,z,p,c)
    
    % INPUTS:
    %   t = simulation time
    %   z = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    %       .l  = pendulum length
    %       .m1 = cart mass
    %       .m2 = pendulum mass
    %       .g  = gravity
    %       .L  = rail length
    %       .s  = max input force (motor torque*radius)
    %       .l_un  = pendulum length with uncertainty
    %       .m1_un = cart mass with uncertainty
    %       .m2_un = pendulum mass with uncertainty
    %   c = control parameter struct
    %       .N = number of sample points
    %       .dt = sample time step size
    %       .Q = error cost
    %       .R = actuation effort  
    %       .K = optimal feedback gain
    %       .S = cost-to-go
    %       .options = options for NLP solver
    % 
    % OUTPUTS:
    %   u = u(t,z) = input as function of time and state feedback
    
    % unpack parameters
    S = c.S;
    K = c.K;
    dt = c.dt;
    u_o = c.u_o;
    zt = p.zt(t);

    % stablize about inverted equilibrium
    % cost-to-go from algebraic Ricatti eqn as a metric of closeness
    if (z-zt)'*S*(z-zt) < 1.0
        u = -K*(z-zt);
    
    % interpolate OCP result if far away from equilibrium
    else
        % find proper i to sample from
        i = ceil(t/dt);

        % trim off ends
        if i == 0
            i = 1;
        end
        if i >= numel(u_o)
            i = numel(u_o) - 1;
        end
        
        % linear interpolation for u
        u = interp1([0 1], [u_o(i) u_o(i+1)], mod(t,dt)/dt);
    end
end

