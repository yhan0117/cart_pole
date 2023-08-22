function dz = dynamics(t,z,c,u,p)

    % This function computes the state derivates of the cart-pole. The
    % cart rolls along horizontal rails. There is no friction in the system.
    % Both the cart and the pole (pendulum) are point masses.
    %
    %
    % INPUTS:
    %   z = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    % OUTPUTS:
    %   dz = dz/dt = time derivative of state
    %   

    % global loop time and decision variables
    global dv_prev t_prev output;

    % extract actual states from z and dv
    N = c.N;
    z = z(1:4);

    % calculate control law
    if t-t_prev > c.dt_p % end of a control loop
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