clc; clear; close all;

xmin=-3; % setting the figure limits
xmax=+3;
ymin=-3;
ymax=+3;

% kr = -0.5; kq = -2;
% kr = 0.5; kq = 2;
%% Control Gains
% kr = -0.1; kq = -1;
% kr = -1; kq = -5;
% kr = 0.5; kq = 1;
%
% kr = 0.1; kq = 0.5;
% k = 0;

%% New pose in paper 
% % positive gains
% X0 = [];
% Y0 = [ ];
% Th0 = [];
% negative gains
% X0 = [ 2.25     0     -1.5       -2.2       -1        2.0       2.0      -2        -2.0       0       2.0          1.0];
% Y0 = [  1       2.5    2.0        0       -2.25      -2.0      0.0      1         -1.5        -2.0     2.0         -2.25];
% Th0 = [-pi/2   pi/3     5*pi/4    pi     -pi/6       -pi/4   -pi/6   3*pi/4    -pi/3         pi/2     -3*pi/4        0];

X0 = [2.25; 2.25; 2.25; -2.0];
Y0 = [1.0; 0.0;  0.0; 1.0];
Th0 = [-pi/2; -pi/2; pi/2; 0.0];


%% old pose in paper 
% X0 = [2   1  -1  -2   -2   -1    1   2];
% Y0 = [1   2   2   1   -1   -2   -2  -1];
% Th0 = [1  2   2   3   -3   -2   -2  -1]*pi/4;
%
% X0 = -0.5; 
% Y0 = 1.25;
% Th0 = 2*pi/4;


% Test
% X0 = 1.3;
% Y0 = -0.5;
% Th0 = 3*pi/2;
% alpha0 = myAtan2(X0,Y0);

% % Positive gains
% X0 = [1.25  0.5   -0.5  -1.25  -1.5  -1.25  -0.5    0.5   1.25  1.5];
% Y0 = [ 1    1.25  1.25    1      0     -1    -1.25  -1.25   -1    0];
% Th0 = [1     2      2     3      4     -3     -2      -2    -1    0]*pi/4;
% KR_positive = [0.1  0.2  0.3  0.4  0.5  0.6];
% KQ_positive = [1.0  0.8  0.6  1.2  1.5  2.0];
% % 
% KR_negative = -[0.1  0.2  0.3  0.4  0.5  0.6];
% KQ_negative = -[1.0  0.8  0.6  1.2  1.5  2.0];


%% Arena plot
% rgb = [160,82,45];
% plot_circle([0; 0],3,rgb)
% xlabel('x (m)'); ylabel('y (m)');

i = 1;
str_plot = {'-*r', '-pb', '-sg', '-hk', '-sm', '-hc', '-*r', '-sg', '-pb', '-vk', '-dm', '-hc'};

for j = 1:length(X0)
    
    x0 = X0(j); y0 = Y0(j); th0 = Th0(j);
    k = 0; c = 0;
    % x0 = 1; y0 = 2; th0 = 1*pi/2;
    % if 0 < th0 && th0 <= pi
    %     th0 = th0 + 2*pi;
    % elseif -pi < th0 && th0 <= 0
    %     th0 = th0 - 2*pi;
    % end
    if j==1 || j==2
        kr = -0.5; kq = -1.5;
        % kr = KR_positive(j); kq = KR_positive(j);
    else
        kr = 0.5; kq = 1.5;
        % kr = KR_negative(j); kq = KR_negative(j);
    end
    
    
    % X0 = [x0; y0; th0];
    x = x0; y = y0; th = th0; t = 0; 
    dt = 0.1; ne = inf; v = 0; w = 0;
    V = [];
    W = [];
    % XX = [];
    YY = [];
    while ne > 0.01 && t < 40
        k = k+1;
        % robot model
        x = x + v*cos(th)*dt;
        y = y + v*sin(th)*dt;
        th = th + w*dt;
        
        %
        p = sqrt(x^2 + y^2);
        alpha = myAtan2(x,y);
        
        %% Position control
        q = sin(th-alpha);
        %***
        % q = sin(th-2*alpha);
        %***
        
        %% Pose control
        %***
        % q = sin(mod(th-alpha,2*pi)-alpha); %% Negative gains
        % q = sin(mod(th-alpha,2*pi))*cos(alpha) - cos(mod(th-alpha,2*pi))*sin(alpha); %% Negative gains
        
        % q = sin(mod(mod(th-alpha,-pi),2*pi)-alpha); %% Positive gains
        %***
        
        % control law
        v = kr*p;
        w = kq*q;
        
        % error
        %****
        % th = mod(th,2*pi);
        %****
        X = [x; y; th];
        % X = [x; y; th];
        ne = norm(X(1:2));
        % ne = norm(X(1:3))
        
        
        t = t + dt;
% %% Plot robot         
%         % plot
%         plot(x + 0.015*rand,y+0.01*rand,'.r','LineWidth',2);
%         xlabel('x (m)'); ylabel('y (m)');
%         grid on; hold on;
%         % title('k_{q}< 0, k_{w} < 0');
%         title('Position Stabilization');
%         
%         % if  (rem(k,20) == 0 || k == 1) && c < 3
%         if  k == 1
%             % draw_robot(x,y,th); grid on; hold on;
%             draw_robot_tb3(x,y,th,k); grid on; hold on;
%             c = c+1;
%         end
%         % axis([xmin xmax ymin ymax]) % setting the figure limits
%         % axis([-2 2 -2 2]);
%         % axis([-1.75 1.75 -1.75 1.75]);
%         drawnow;
%     
%         % Movie
%     % Mov(i) = getframe;
%     % clf;
%     % i = i + 1;

%% Plot convergence 
% plot(t,p,str_plot{j}); hold on; grid on; drawnow;

%% Plot convergence 
% if j == 1
    plot(2*t,p,str_plot{j},'LineWidth',3); hold on; grid on; drawnow;
%    plot(t,w,'-pg','LineWidth',2); hold on; grid on; drawnow;
% elseif j==2
%     plot(t,v,'-sb','LineWidth',2); hold on; grid on; drawnow;
%     plot(t,w,'-pk','LineWidth',2); hold on; grid on; drawnow;
% end
%     
    
    
    %% Positions
    % XX = [XX; t x];
    YY = [YY; t y];
    
    %% Velocities
%     V = [V; t v];
%     W = [W; t w];
    end
%  t   
%     %% Plot Control Inputs
%     plot(V(:,1),V(:,2),str_plot{j}); hold on; grid on;
%     xlabel('t (s)'); ylabel('v (m/s)');
%     legend('Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6');
%     
%     plot(W(:,1),W(:,2),str_plot{j}); hold on; grid on;
%     xlabel('t (s)'); ylabel('w (rad/s)');
%     legend('Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6');
% 
%     plot(XX(:,1),XX(:,2),str_plot{j}); hold on; grid on;
%     xlabel('t (s)'); ylabel('x (m)');
%     legend('Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6');

%%%%
%     plot(YY(:,1),YY(:,2),str_plot{j}); hold on; grid on;
%     xlabel('t (s)'); ylabel('x (m)');
%     legend('Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6');

end
% title('k_{q}>0, k_{w}>0');
xlabel('t (s)'); ylabel('\rho (m)')
title('Distance to Target');
% 
% rgb = [46,139,87];
% plot(0,0,'*','Color',rgb/255,'LineWidth',2); hold on

% % %---
% % th0*180/pi
% % Th_final = th*180/pi
% % alpha0*180/pi
% 
% % Mov(i) = getframe;
% % 
% % video = VideoWriter('PoseControl.mp4','MPEG-4');
% % open(video)
% % writeVideo(video,Mov);
% % close(video)
