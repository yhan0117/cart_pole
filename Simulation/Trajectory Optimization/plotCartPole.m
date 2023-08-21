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

function plotCartPole(t,z)

    % INPUTS:
    %   t = [1, n] = time stamps for each state vector
    %   z = [4, n] = [x;q;dx;dq] = state of the system
    %
    % OUTPUTS:
    %   a simple plot for each state of the system over the simulation

    %%%% Unpack the state:
    x = z(1,:);
    q = z(2,:);
    dx = z(3,:);
    dq = z(4,:);

    %%%% Plots:
    subplot(2,2,1);
    plot(t,x)
    ylabel('x [m]')
    title('Position')

    subplot(2,2,2);
    plot(t,mod(q,2*pi)*180/pi)
    ylabel('theta [rad]')
    title('Angle')

    subplot(2,2,3);
    plot(t,dx)
    ylabel('v [m/s]')
    title('Velocity')

    subplot(2,2,4);
    plot(t,dq)
    ylabel('\omega [rad/s]')
    title('Angle Rate')

end