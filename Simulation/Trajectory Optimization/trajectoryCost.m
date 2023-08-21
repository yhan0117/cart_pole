% Computes the cost and cost jacobian
function g = trajectoryCost(z, c)
    
    %%%% INPUT
    % z: decision variable vector
    % N: number of sample points
    % nx: dimension of state vector, x
    % nu: dimension of input vector, u
    % dt: time step duration
    
    %%%% OUTPUT
    % g: total accrued cost; scalar
    % dG: gradient of total accrued cost; nz by 1 vector

    %%%% unpack constants
    N = c.N;
    nx = c.nx;
    nu = c.nu;
    dt = c.dt;
    x_f = c.x_f;
    %%%% init
    g = 0;
    dG = zeros(N*(nx + nu),1);

    %%%% trajectory cost based on actuator effort
    dG(5:5:end) = 2*z(5:5:end);
    
    
    % locate u in z
    u = z(5:5:end);
    x = [z(1:5:end) z(2:5:end) z(3:5:end) z(4:5:end)]';
    % Cost by magnitude of input
    Q = diag([1 1 1 1]);
    R = 1;
    rho = 100.5*Q;
    x_end = x(:,end);
    JQ = 0.5*(x(:,1)-x_f).'*(x(:,1)-x_f);

    for i = 2:N-1
        JQ = JQ + (x(:,i)-x_f).'*Q*(x(:,i)-x_f);
    end

    JR = 0.01*sum(u'*R*u);
    Jt = sum((x_end-x_f).'*rho*(x_end-x_f));
    % fprintf("%.3e %.3e %.3e\n", JQ, JR, Jt);
    % total cost
    g = JR;%+JQ+Jt;
end