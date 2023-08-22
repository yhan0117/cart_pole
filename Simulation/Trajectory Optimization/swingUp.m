% executes a direct shooting optimization program to find an input 
% sequence to drive the cartpole system from z0 to zf.
function u_o = swingUp(p,c)
    
    %%%% INPUT:
    % z0: state at the start; 4 by 1 vector
    % zf: state at the end; 4 by 1 vector
    
    %%%% OUTPUT:
    % u: decision variable vector containing u_i at each timestep, N x 1
    
    %%%% unpack parameters
    s = p.s;
    N = c.N;
    options = c.options;

    %%%% input bounds
    lb = repmat(-s,N,1);
    ub = repmat(s,N,1);

    %%%% initial guess
    u0 = zeros(N, 1);
    
    %%%% optimization problem struct     
    [u_o,~] = fmincon(@(u)(u'*u),u0,[],[],[],[],lb,ub,@(u) dynamicsConstraints(u,p,c), options);
end