% This function represents the control law that computes the input
% based on full state feedback
function u = control(t,z,p,c,ctrl)

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
    %       .Ke = swing up control gain
    %       .Q = error cost
    %       .R = actuation effort  
    %       .dt_p = control loop interval          
    %       .K = control gains from LQR
    %       .S = cost to go from algebraic Riccati equation
    %   ctrl = controller option
    %          0 - Energy shaping
    %          1 - Resonance based
    %
    % OUTPUTS:
    %   u = u(t,z) = input as function of time and state feedback

    % unpack parameters
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration
    K = c.K;
    Ke = c.Ke;
    zt = c.zt(t);
    global z_prev

    % stablize about inverted equilibrium
    if abs(mod(z(2),2*pi) - pi) < 25*pi/180  % if near top
            u = -K*(z-zt);

    % swing up
    else 
        % Swing up option 1
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
    
        % Swing up option 2
        else
            dx = sign(z(4)).*abs(z(4)^0.3);
            u = -Ke*sqrt(abs(z_prev))*dx*cos(z(2)); % u out of phase angle and minimized near 90 degrees 

            % bump start
            if abs(z(2)) < 1e-3 && abs(z(4)) < 1e-3
                u = 0.3;
            end
        end
    end

end

