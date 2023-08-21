% calculates the indices of z where x_i and u_i are located
% such that z(x_i_inds) = x_i and z(u_i_inds) = u_i.
function [x_i_inds, u_i_inds] = z_index(i, nx, nu)

    % i: sample number; scalar
    % nx: dimension of state vector, x; scalar
    % nu: dimension of input vector, u; scalar
    %
    % x_i_inds: indices such that z(x_i_inds) = x_i; 1 by n_x vector
    % u_i_inds: indices such that z(u_i_inds) = u_i; 1 by n_u vector

    x_i_inds = (i-1)*(nx+nu)+1:i*(nx+nu)-nu;
    u_i_inds = i*(nx+nu)-nu+1:i*(nx+nu);
end

