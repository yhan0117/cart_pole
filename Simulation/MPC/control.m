function [dv_o,output] = control(t,z,p,c,opt)

    %%%%
    % This function represents the control law that computes the input
    % based on full state feedback
    %
    % INPUTS:
    %   z = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    %       .g  = gravity
    %       .m1 = cart mass
    %       .m2 = pole mass
    %       .l  = pendulum length
    %       .L  = rail length
    %       .s  = max input force
    %   c = control parameter struct
    %       .z_t = target state
    %       .rho = terminal cost
    %       .Q   = error cost matrix
    %       .R   = actuation effort cost matrix
    %       .N   = control horizon 
    %       .t_c = control/ prediction time stamps
    %   opt = ode and optimziation solver options
    %       .ODE = options for ode45 
    %       .NLP = options for fmincon
    %
    % OUTPUTS:
    %   u = u(z) = input as function of current states

    % control horizon     
    N = c.N;        

    % initial state of each mpc iteration => current position wrt real time
    z0 = z; 
    global dv_prev
    dv0 = dv_prev;   % 5 x N
    
    %%%% boundary constraints
    lb = -inf(5*N,1);
    ub = inf(5*N,1);

    % max rail length
    lb(1:4:4*N-3) = repmat(-p.L/2,N,1);
    ub(1:4:4*N-3) = repmat(p.L/2,N,1);

    % cart force
    lb(4*N+1:end) = repmat(-p.s,N,1);
    ub(4*N+1:end) = repmat(p.s,N,1);

    % initial condition -> 4 equalities require 4 rows
    Aeq = zeros(4, 5*N);
    Aeq(1:4, 1:4) = eye(4);
    beq = z0;
    
    %%%% SQP approach of trajctory optimization 
    % nonlincon is appended to Lagrangian as part of objective function 
    % => may not be satisfied 
    % => add weight (which indirectly adds to cost) to make more siginificant
    [dv_o,~,~,output] = fmincon(@(dv) trajCost(dv,t,c,p),dv0,[],[],Aeq,beq,lb,ub,@(dv) collocationCon(dv,p,c), opt);

end

