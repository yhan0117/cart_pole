function dz = eom(z,u,p)
    %%%% unpack parameters
    l = p.l;    % pendulum length
    M = p.M;    % cart mass
    m = p.m;    % pole mass
    g = p.g;    % gravity 

    %%%% equations of motion
    dz = zeros(4,1);

    delta = m*sin(z(2))^2 + M;
	dz(1) = z(3);
	dz(2) = z(4);
	dz(3) = m*l*(z(4)^2)*sin(z(2))/delta + m*l*g*sin(z(2))*cos(z(2))/delta/l + u/delta;
	dz(4) = -m*(z(4)^2)*sin(z(2))*cos(z(2))/delta - (m+M)*g*sin(z(2))/delta/l - u*cos(z(2))/delta/l;
end