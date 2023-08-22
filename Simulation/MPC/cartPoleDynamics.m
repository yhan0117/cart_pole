% This function computes the state derivates of the cart-pole. The
% cart rolls along horizontal rails. There is no friction in the system.
% Both the cart and the pole (pendulum) are point masses.
function dz = cartPoleDynamics(t,z,c,u,p)

    % INPUTS:
    %   t = simulation time 
    %   z = [x;q;dx;dq] = state of the system
    %   u = feedback control law
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
    %
    % OUTPUTS:
    %   dz = dz/dt = time derivative of state

    % global loop time and decision variables
    global dv_prev t_prev output;

    % extract actual states from z and dv
    N = c.N;
    z = z(1:4);

    % calculate control law
    if t-t_prev > c.dt_p % wait until next control loop
        disp(t)
        [dv_prev, out] = u(t,z);
        u = (dv_prev(4*N+1)+dv_prev(4*N+2))/2;
        output = [output;out];
        t_prev = t;

        % reset guess if converges to infeasible point
        if out.constrviolation > 1e-9
            zt = c.zt(t);
            for i=1:N
                dv_prev(4*i-3:4*i) = z + (i-1)*(zt(1:4)-z)/(N-1);  % linear interpolation 
            end
        end
    else
        u = (dv_prev(4*N+1)+dv_prev(4*N+2))/2;
    end
    
    % saturation
    if abs(u) > p.s
        u = sign(u)*p.s;
    end

    % external disturbance
    if t > 2 && t < 2+c.t_d
        dz = [eom(z,u+c.ui,p);u]; % append with u to also keep track of input trajectory
    else    
        dz = [eom(z,u,p);u];
    end
end

% Equations of motion to plug in Runge Kutta solver
function dz = eom(z,u,p)
    
    % unpack parameters (uncertainty included)
    l = p.l_un;     % pendulum length
    M = p.m1_un;    % cart mass
    m = p.m2_un;    % pole mass
    g = p.g;        % gravity 
    
    % initialize
    dz = zeros(4,1);

    % z' = f(z,u)
    delta = m*sin(z(2))^2 + M;
	dz(1) = z(3);
	dz(2) = z(4);
	dz(3) = m*l*(z(4)^2)*sin(z(2))/delta + m*l*g*sin(z(2))*cos(z(2))/delta/l + u/delta;
	dz(4) = -m*(z(4)^2)*sin(z(2))*cos(z(2))/delta - (m+M)*g*sin(z(2))/delta/l - u*cos(z(2))/delta/l;
end
