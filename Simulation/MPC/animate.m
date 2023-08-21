function animate(z,p,r)
    clearvars -global -except dv_prev t_prev output
    disp("Animation in progess!!!")

    % animation parameters
    record = r.record;
    filename = r.filename;
    fps = r.fps;        % simulation resolution
    t_s = r.t_s;

    % Convert states to cartesian positions:
    pos = cartPolePosition(z,p);
    x1 = pos(1,:);
    y1 = pos(2,:);
    x2 = pos(3,:);
    y2 = pos(4,:);
    
    % Plotting parameters:
    p.w = 0.6*p.l;  %Width of the cart
    p.h = 0.4*p.l;  %Height of the cart
    p.r = 0.1*p.l;  % Radius of the pendulum bob
    
    % Compute the extents of the drawing, keeping everything in view
    padding = 0.2*p.l;  % Free space around edges
    if max(abs(x1)) > 0.7 && max(x1) - min(x1) < 0.7
        center = (min(min(x1 - 0.5*p.w,  x2 - p.r)) + max(max(x1 + 0.5*p.w,  x2 + p.r)))/2;
        xLow = center - 0.9;
        xUpp = center + 0.9;
    else
        xLow = -0.75;
        xUpp = 0.75;
    end
    yLow = min(min(y1 - 0.5*p.h,  y2 - p.r)) - padding;
    yUpp = max(max(y1 + 0.5*p.w,  y2 + p.r)) + padding;
    extents = [xLow,xUpp,yLow,yUpp];
    
    % Set up figure:
    hold on;    %  <-- This is important!
    set(gcf,'DoubleBuffer','on');   % Prevents flickering (maybe??)
    
    if record
        for k = 1:length(t_s)
            % Redraw the image
            drawCartPole(t_s(k),pos(:,k),extents,p);

            % Saving the figure
            frame = getframe(gcf);

            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if k == 1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime', 1/fps);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', 1/fps);
            end
        end
    else
        time = 0;
        tic;
        while time <= t_s(end)
        
            % Compute the position of the system at the current real world time
            posDraw = interp1(t_s',pos',time')';
            
            % Redraw the image
            drawCartPole(time,posDraw,extents,p);
        
            % Update current time
            time = toc;
        end
    end
end

