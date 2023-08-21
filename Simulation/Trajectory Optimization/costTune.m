clc;clear
de0 = [10 10 10 10 10];
problem.objective = @(de) cost(de);

% options for optimization funciton
% --> manually set gradient of constraint and objective functions
options = optimoptions('fmincon','Algorithm','sqp','UseParallel', 'always', 'display', 'iter-detailed','MaxFunctionEvaluations',100000);

problem.options = options;
problem.solver = 'fmincon';

% nonlinear constraints and their gradients 
problem.nonlcon = [];

% initial guess
problem.x0 = de0;

% linear constraints and bounds
problem.Aeq = [];
problem.beq = [];
problem.lb = 0.01*ones(1,5);
problem.ub = [];

%%%% optimization results
z = fmincon(problem)


function J = cost(de)
    Q = diag(de(1:4));
    R = de(5);
    load('OptTrajWConstraint.mat');
    x_f = [0 pi 0 0]';
    JQ = 0;
    for i = 1:numel(u)-1
        JQ = JQ + (z_(:,i)-x_f).'*Q*(z_(:,i)-x_f);
    end
    JR = R*sum(u.^2);
    J = JR+JQ;
end