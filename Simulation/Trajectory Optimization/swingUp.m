% Solves an OCP to drive the system from z0 to zt.
function u_o = swingUp(p,c)
    
    % INPUTS:
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
    %   u_o: optimal input sequence, N x 1
    
    % unpack parameters
    s = p.s;
    N = c.N;
    options = c.options;

    % input bounds
    lb = repmat(-s,N,1);
    ub = repmat(s,N,1);

    % initial guess
    u0 = zeros(N, 1);
    
    % solves through direct shooting     
    [u_o,~] = fmincon(@(u)(u'*u),u0,[],[],[],[],lb,ub,@(u) nonlinCon(u,p,c), options);
end