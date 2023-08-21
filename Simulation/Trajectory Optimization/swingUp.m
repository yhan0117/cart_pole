% executes a direct collocation optimization program to find an input 
% sequence to drive the cartpole system from x_0 to x_f.
function [z, Aeq, beq, lb, ub, z0] = swingUp(x_0, p, c)
    
    %%%% INPUT:
    % x_0: state at the start; n_x by 1 vector
    % x_f: state at the end; n_x by 1 vector
    
    %%%% OUTPUT:
    % z: decision variable (column) vector containing the x_i and u_i at each timestep
    % N*(nx+nu) by 1

    %%%% nomenclature
    % Aeq: matrix for linear constraint Aeq z = beq
    % beq: (column) vector for linear constraint Aeq z = beq
    % lb: lower bound of z lb <= z <= ub; n_z by 1 vector
    % ub: upper bound of z lb <= z <= ub; n_z by 1 vector
    % z0: initial guess for z; n_z by 1 vector
    
    %%%% unpack parameters
    L = p.L;
    s = p.s;
    N = c.N;
    x_f = c.x_f;
    nx = c.nx;
    nu = c.nu;

    %%%% Boundary conditions and linear equality constraints
    Aeq = zeros(2*nx, N * (nx + nu));
    beq = zeros(2*nx, 1);

    Aeq(1:nx, 1:nx) = eye(nx);
    Aeq(nx+1:2*nx, (N-1)*(nx+nu)+1:(N-1)*(nx+nu)+nx) = eye(nx);
    
    beq(1:nx) = x_0;
    beq(nx+1:2*nx) = x_f;

    %%%% Upper and lower bounds
    lb = -inf(N * (nx + nu),1);
    ub = inf(N * (nx + nu),1);

    for i=1:N
        % Find indices of z that correponds to time step i
        [x_i_inds,u_i_inds] = z_index(i, nx, nu);

        % Saturation [-M, M] for input u_i at each time step
        lb(u_i_inds) = -s;
        ub(u_i_inds) = s;

        % Set rail lenght limits at each time step
        lb(x_i_inds(1)) = -0.9*L/2;
        ub(x_i_inds(1)) = 0.9*L/2;
    end

    % Initial guess for z
    z0 = zeros(N * (nx + nu), 1);

    for i=1:N
        % ith sample by linear interpolation between end points
        [x_i_inds, ~] = z_index(i, nx, nu);
        z0(x_i_inds) = x_0 + (i-1)*(x_f-x_0)/(N-1);
    end
    
    z0 = c.z0;

    %%%% optimization problem struct 

    % objective function to minimize --> trajectory cost
    problem.objective = @(z) trajectoryCost(z, c);

    % options for optimization funciton
    % --> manually set gradient of constraint and objective functions
    % options = optimoptions('fmincon','SpecifyObjectiveGradient',false,'SpecifyConstraintGradient',true,'Algorithm','sqp', 'display', 'iter-detailed','MaxFunctionEvaluations',100000);
    options = optimoptions('fmincon','SpecifyObjectiveGradient',false,'SpecifyConstraintGradient',true, 'UseParallel', 'always','Algorithm','interior-point', 'display', 'iter-detailed','MaxFunctionEvaluations',10000,'EnableFeasibilityMode',true);

    problem.options = options;
    problem.solver = 'fmincon';
    
    % nonlinear constraints and their gradients 
    problem.nonlcon = @(z) nonLinConstraints(z, p, c);

    % initial guess
    problem.x0 = z0;

    % linear constraints and bounds
    problem.Aeq = Aeq;
    problem.beq = beq;
    problem.lb = lb;
    problem.ub = ub;
 
    % rng default % For reproducibility
    % 
    % ms = MultiStart('UseParallel', 'always');
    % [z,f] = run(ms,problem,5);
    %%%% optimization results
    z = fmincon(problem);
end

% calculates all nonlinear constraints
function [c, ceq, dC, dCeq] = nonLinConstraints(z, p, c)

    % nonlinear equality constraints and their gradients (dynamics)
    [ceq, dCeq] = dynamicsConstraints(z, p, c);

    % nonlinear inequality constraints --> none
    c = zeros(0,1);
    dC = zeros(0,numel(z));
    
    % sparse to improve computation speed, can also be dense
    dC = sparse(dC)';
    dCeq = sparse(dCeq)';
end