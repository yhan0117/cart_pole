function [C,Ceq] = dynamicsConstraints(u,p,c)
    
    % unpack parameters
    N = c.N;
    dt = c.dt;

    z0 = p.z0;
    L = p.L;
    zt = p.zt(0);

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