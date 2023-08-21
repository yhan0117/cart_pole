function dz = eom(z,u,p)

    % Equations of motion to plug in Runge Kutta solver
    % INPUTS:
    %   z: current state
    %   u: control input
    %   p: parameter struct
    % OUTPUTS:
    %   dz = dz/dt = time derivative of states
    
    % Unpack the physical palametels
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration
    
    % initialize
    dz = zeros(4,1);

    % z' = f(z,u)
    delta = m*sin(z(2))^2 + M;
	dz(1) = z(3);
	dz(2) = z(4);
	dz(3) = m*l*(z(4)^2)*sin(z(2))/delta + m*l*g*sin(z(2))*cos(z(2))/delta/l + u/delta;
	dz(4) = -m*(z(4)^2)*sin(z(2))*cos(z(2))/delta - (m+M)*g*sin(z(2))/delta/l - u*cos(z(2))/delta/l;
end