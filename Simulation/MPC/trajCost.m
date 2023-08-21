function J = trajCost(dv,t,c,p)
    
    %%%%
    % cost function for piecewise constant control
    % 
    % INPUTS:
    %   dv = vector of decision variables, N x 5
    %   c = control parameter struct
    %       .z_t = target state
    %       .rho = terminal cost
    %       .Q   = error cost matrix
    %       .R   = actuation effort cost matrix
    %       .N   = control horizon 
    %       .t_c = control/ prediction time stamps
    %   
    % OUTPUTS:
    %   J = scalar total cost
        
    % Unpack the control parameters         
    zt = c.zt(t);   % target state (time variant)
    T = c.T';        % terminal cost
    Q = c.Q;        % error cost matrix
    R = c.R;        % actuation effort cost matrix
    N = c.N;        % control horizon 

    % extract from decision variables
    % state vector, 4 x N
    % control vector, 1 x N 
    z = zeros(4, N);
    u = zeros(1, N);

    for i = 1:N
        z(:,i) = dv(4*i-3:4*i);
        u(i) = dv(4*N+i);
    end

    if c.mode
        % error 
        e = z - zt;
        
        % Cost function
        % QR form + terminal cost => explicit function of decision variables
        JQ = sum(Q*(e.*e)*T, 'all');
        JR = sum(R*(u.*u), 'all');
        
        % total cost
        J = JQ+JR;
    else
        % get pendulum position
        pos = cartPolePosition(z,p);
        e = [z; pos(1:2,:)] - zt;
        
        JQ = sum(Q*(e.*e)*T, 'all');
        JR = sum(R*(u.*u), 'all');        
        J = JQ+JR;
    end

    % gradient of J
end