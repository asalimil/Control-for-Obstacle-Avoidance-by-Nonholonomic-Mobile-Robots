clc; clear; close all;

% kr = 0.1; kq = 1;
% kr = -0.1; kq = -1;
kr = -0.5; kq = -2;
% kr = 0.1; kq = 0.5;
% kr = 0.1; kq = 0.5;
% k = 0;

X0 = [1.5 0.5 -0.5  -1.5  -1.75  -1.5  -0.5   0.5  1.5  1.75];
Y0 = [ 1  1.5  1.5    1     0     -1   -1.5  -1.5   -1   0];
Th0 = [1   2    2     3     4     -3    -2    -2    -1   0]*pi/4;


% arena plot
% rgb = [160,82,45];
% plot_circle([0; 0],1.5,rgb)
% xlabel('x (m)'); ylabel('y (m)');

i = 1;
for j = 1:length(X0)
    
    x0 = X0(j); y0 = Y0(j); th0 = Th0(j);
    k = 0; c = 0;
    % x0 = 1; y0 = 2; th0 = 1*pi/2;
    % if 0 < th0 && th0 <= pi
    %     th0 = th0 + 2*pi;
    % elseif -pi < th0 && th0 <= 0
    %     th0 = th0 - 2*pi;
    % end
    
    % X0 = [x0; y0; th0];
    x = x0; y = y0; th = th0; t = 0; dt = 0.1; ne = inf; v = 0; w = 0;
    
    while ne > 0.025
        k = k+1;
        % robot model
        x = x + v*cos(th)*dt;
        y = y + v*sin(th)*dt;
        th = th + w*dt;
        
        %
        p = sqrt(x^2 + y^2);
        alpha = myAtan2(x,y);
        q = sin(th-alpha -th0);
        
        % control law
        v = kr*p;
        w = kq*q;
        
        % error
        X = [x; y; th];
        ne = norm(X(1:2));
        
        t = t + dt;
        
        % plot
        plot(x,y,'.r','LineWidth',2);
        xlabel('x (m)'); ylabel('y (m)');
        grid on; hold on;
        
        if  (rem(k,50) == 0 || k == 1)
            % draw_robot(x,y,th); grid on; hold on;
            draw_robot_tb3(x,y,th,k); grid on; hold on;
            c = c+1;
        end
        axis([-2 2 -2 2]);
        drawnow;
    
%     % Movie
%     Mov(i) = getframe;
%     i = i + 1;
    end
    % clf;
    
end

rgb = [46,139,87];
plot(0,0,'*','Color',rgb/255,'LineWidth',2); hold on

% Mov(i) = getframe;
% 
% video = VideoWriter('PositionControl.mp4','MPEG-4');
% open(video)
% writeVideo(video,Mov);
% close(video)
