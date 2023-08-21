function u = control(t,z,p,c,ctrl)

    % This function represents the control law that computes the input
    % based on full state feedback
    %
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
    %       .Ke = gain for energy control
    %       .zt = target state (time variant)
    %
    % OUTPUTS:
    %   u = u(t,z) = input as function of time and state feedback

    % unpack the physical parameters
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration

    % unpack the control parameters
    S = c.S;
    K = c.K;
    Ke = c.Ke;
    zt = c.zt(t);
    global z_prev

    % stablize about inverted equilibrium

    % if (z-zt)'*S*(z-zt) < 8.0 % if near top
    if abs(mod(z(2),2*pi) - pi) < 25*pi/180  % if near top
            u = -K*(z-zt);

    % swing up
    else 
        % Lyapunov option 1
        if ctrl
            % energy at homoclinic orbit
            Ed = m*g*l;
    
            % energy at current state
            Ec = 0.5*m*l^2*z(4)^2 - m*g*l*cos(z(2));
    
            % energy error
            Ee = Ec - Ed;
        
            % input acceleration A of cart
            A = Ke*Ee*cos(z(2))*z(4);
        
            % convert A to u (force)
            u = A*(m*sin(z(2))^2 + M) - m*l*(z(4)^2)*sin(z(4)) - m*g*sin(z(4))*cos(z(4));
    
        % Lyapunov option 2
        else
            dx = sign(z(4)).*abs(z(4)^0.3); % 
            u = -Ke*sqrt(abs(z_prev))*dx*cos(z(2)); % u out of phase angle and minimized near 90 degrees 

            % bump start
            if abs(z(2)) < 1e-3 && abs(z(4)) < 1e-3
                u = 0.3;
            % brake
            % elseif mod(abs(pi-z(2)),2*pi) < pi/3 && z(2)*z(4) > 0
            %     u = u - sign(u)*(z(2)*z(4))^1.5;
            end
    
        end
    end

end

