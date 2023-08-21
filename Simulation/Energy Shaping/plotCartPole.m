function plotCartPole(t,z,u)

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
    subplot(2,6,1:3);
    plot(t,x)
    ylabel('x [m]')
    title('Position')

    subplot(2,6,4:6);
    plot(t,mod(q,2*pi)*180/pi)
    ylim([0,360])
    ylabel('\theta [degree]')
    title('Angle')

    subplot(2,6,7:8);
    plot(t,dx)
    ylabel('v [m/s]')
    title('Velocity')
 
    subplot(2,6,9:10);
    plot(t,dq)
    ylabel('\omega [rad/s]')
    title('Angle Rate')

    subplot(2,6,11:12);
    plot(t,u)
    ylabel('u [N]')
    title('Input Force')
end