function dz = cartPoleDynamics(t,z,c,u,p)

    % This function computes the state derivates of the cart-pole. The
    % cart rolls along horizontal rails. There is no friction in the system.
    % Both the cart and the pole (pendulum) are point masses.
    %
    % INPUTS:
    %   z = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    % 
    % OUTPUTS:
    %   dz = dz/dt = time derivative of state
    %

    % global loop time and decision variables
    global u_prev t_prev z_prev

    % extract actual states from z and dv
    z = z(1:4);
    
    % control law
    if t-t_prev > c.dt_p % end of a control loop
        u = u(t,z);
        u_prev = u;
        t_prev = t;
    else
        u = u_prev;
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
    z_prev = dz(4);
end