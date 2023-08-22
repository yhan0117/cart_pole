function u = control(t,z,p,c,u_o)

    % This function represents the control law that computes the input
    % based on full state feedback
    %
    % INPUTS:
    %   z = [x;q;dx;dq] = state of the system
    %   p = physical parameter struct
    %       .g = gravity
    %       .m1 = cart mass
    %       .m2 = pole mass
    %       .l = pendulum length
    %   c = control parameter struct 
    %       .K = control gains from LQR
    %       .S = cost to go from algebraic Riccati equation
    %       .zt = target state (time variant)
    %
    % OUTPUTS:
    %   u = u(t,z) = input as function of time and state feedback

    % unpack physical and test parameters
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration
    zt = p.zt(t);

    % unpack the control parameters
    S = c.S;
    K = c.K;
    dt = c.dt;

    % stablize about inverted equilibrium
    % cost-to-go from algebraic Ricatti eqn as a metric of closeness
    if (z-zt)'*S*(z-zt) < 1.0
        u = -K*(z-zt);
    
    % interpolate from D.C. if far away from equilibrium
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

