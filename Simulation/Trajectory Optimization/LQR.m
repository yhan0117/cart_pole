function [K, S] = LQR(p)
    %%%% Unpack the physical palametels
    l = p.l;  % Pendulum length
    M = p.M; % Cart mass
    m = p.m; % Pole mass
    g = p.g;  % Gravity acceleration

    %%%% linearized model
    A = [0 0            1  0;
         0 0            0  1;
         0 m*g/M        0  0;
         0 (m+M)*g/l/M  0  0];
    B = [0 0 1/M 1/M/l].';

    %%%% cost matrices
    Q = diag([0.06,10,1,2]);

    R = 1;

    %%%% calculate optimal gains
    [K, S] = lqr(A,B,Q,R);
end

