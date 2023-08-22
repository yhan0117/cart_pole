% The MIT License (MIT)
%
% Copyright June, 2023 Mohamed Khalid M Jaffar, Universtiy of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

function pos = cartPolePosition(z,p)

    % This function computes the position of the cart and the pole, given the
    % state of the system
    %
    % INPUTS:
    %   z = [4, n] = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    %       .g = gravity
    %       .m1 = cart mass
    %       .m2 = pole mass
    %       .l = pendulum length
    % OUTPUTS:
    %   pos = [4, n] = [x1;y1;x2;y2]; = position of [cart; pole]
    %

    %%%% unpack the state
    x = z(1,:);   %Cart position (Not used in dynamics)
    q = z(2,:);   % pendulum (pole) angle, measure from gravity vector

    %%%% Unpack the physical parameters
    l = p.l;  %Pendulum length

    %%%% Position of the cart:
    x1 = x;
    y1 = zeros(size(x));

    %%%% Position of the pole:
    x2 = x1 + l*sin(q);
    y2 = y1 - l*cos(q);

    %%%% Pack up position vector:
    pos = [x1;y1;x2;y2];

end