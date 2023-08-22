% Nonlinear constraints on terminal state and system limits 
function [C,Ceq] = nonlinCon(u,p,c)
    
    % INPUTS:
    %   u = decision variable vectory, N x 1
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
    %       .z0 = initial state
    %       .zt(t) = target state/ reference signal
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
    %   Ceq = equality constraints
    %   C   = inequality constraints

    % unpack parameters
    N = c.N;
    dt = c.dt;
    z0 = p.z0;
    L = p.L;
    zt = p.zt(0);

    % forward simulation
    % initial state at each ode call 
    y0 = z0;

    % predicted states vector
    zp = zeros(N,4);
    
    % integrate through each time step (constant control input)
    % -> avoid using ode45 only which may lead to discontinuous gradient 
    for i = 1:N
        [~,y] = ode45(@(t,x)eom(x,u(i),p), dt*[i-1 i], y0);
        
        % set the init cond of next time step as result of current step
        y0 = y(end,:);
    
        % update state vector
        zp(i,:) = y0;
    end

    % Set rail lenght limits at each time step
    C(1:N) = zp(:,1) - 0.9*L/2;
    C(N+1:2*N) = -zp(:,1) - 0.9*L/2;

    % terminal constraint
    Ceq = 100*(zt - zp(end,:));
end

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
