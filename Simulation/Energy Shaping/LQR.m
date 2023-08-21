function [K,S] = LQR(p,c)
    
    % unpack the physical parameters
    l = p.l;  % Pendulum length
    M = p.m1; % Cart mass
    m = p.m2; % Pole mass
    g = p.g;  % Gravity acceleration

    % unpack control parameters
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