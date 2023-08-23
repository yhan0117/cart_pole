% Calculate LQR gains with model linearized about top fixed point
function [K,S] = LQR(p,c)
    
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
    %   c = control parameter struct
    %       .Ke = swing up control gain
    %       .Q = error cost
    %       .R = actuation effort  
    %       .dt_p = control loop interval
    % 
    % OUTPUTS:
    %   K = optimal gains
    %   S = cost-to-go from Algebraic Ricatti Eqn

    % unpack parameters
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration
    Q = c.Q;
    R = c.R;

    % jacobian about upward eq    
    A = [0 0            1  0;
         0 0            0  1;
         0 m*g/M        0  0;
         0 (m+M)*g/l/M  0  0];
    B = [0 0 1/M 1/M/l].';


    [K,S] = lqr(A,B,Q,R);
end