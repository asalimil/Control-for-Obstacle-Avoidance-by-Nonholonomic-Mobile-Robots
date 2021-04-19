clc; clear; close all;

% OK
% kr = 5; kq = 12.5;
% kr = 2; kq = 5; % OK

%% positive gains
kr = 2; kq = 5;

%% negative gains
% kr = -2; kq = -5;

% kr = 0.1; kq = 0.5;
% kr = 0.1; kq = 0.5;
% k = 0;

%% positive gains
X0 =  [-0.5   -3.5     -2   -1];
Y0 =  [+3.5   +2.5     -1   +2];
Th0 = [+0.0   -1.0     -1   -2]*pi/4;

%% negative gains
% % X0 =  [0     -2      -1.5          -3];
% % Y0 =  [3    +3.5      +2            0];
% % Th0 = [3*pi/4   +2*pi/3    -3*pi/4  0];
% 
% % %% Negative Gains
% % % Arena plot
% origin_arena = [-1.5; 1.5]; r_arena = 3;
% % % % rgb = [160,82,45];
% % % boundarycolor = [160,82,45];
% % % fill_obs = 0;
% % % plot_circle(origin_arena,r_arena,boundarycolor,fill_obs)
% % % xlabel('x (m)'); ylabel('y (m)');
% % 
% % % plot obstacles
% origin1 = [-1.5; -0.5]; r1 = 0.75;
% % % % rgb = [160,82,45];
% % % fill_obs = 1;
% % % plot_circle(origin1,r1,boundarycolor,fill_obs);
% % 
% % % % rgb = [160,82,45];
% % % fill_obs = 1;
% origin2 = [0.5; 1.5]; r2 = 0.5;
% % % plot_circle(origin2,r2,boundarycolor,fill_obs);
% % 
% origin_ellipsoid = [-3; 2];
% r_small = 1.25; r_big = 0.75;
% a = r_small; b = r_big;
% % % rot_elip = 0*pi/4;
% % % plot_ellipsoid(origin_ellipsoid,r_small,r_big,rot_elip);
% % % % R = [cos(rot_elip)   -sin(rot_elip)  0
% % % %      sin(rot_elip)   +cos(rot_elip)  0
% % % %      0           0         1];
% % 
% % % origin3 = [-1; 2]; r3 = 2*0.5;
% % % plot_circle(origin3,r3);
% % % plotres = [200,    200,  150,   150];
% % % Dt =      [0.05,   0.1,  0.02,  0.05];
% Dt =      [0.2,   0.2,  0.2,  0.2];

% %% positive gains
% % Arena plot
origin_arena = [-1.5; 1.5]; r_arena = 3;
rgb = [160,82,45];
boundarycolor = [160,82,45];
fill_obs = 0;
plot_circle(origin_arena,r_arena,boundarycolor,fill_obs)
xlabel('x (m)'); ylabel('y (m)');

% plot obstacles
origin1 = [-2.25; 3]; r1 = 0.75;
rgb = [160,82,45];
fill_obs = 1;
plot_circle(origin1,r1,boundarycolor,fill_obs);

% rgb = [160,82,45];
% fill_obs = 1;
origin2 = [0; 2]; r2 = 0.5;
% plot_circle(origin2,r2,boundarycolor,fill_obs);

origin_ellipsoid = [-2.5; 0.25];
r_small = 1.25; r_big = 0.75;
a = r_small; b = r_big;
% rot_elip = 0*pi/4;
% plot_ellipsoid(origin_ellipsoid,r_small,r_big,rot_elip);
% R = [cos(rot_elip)   -sin(rot_elip)  0
%      sin(rot_elip)   +cos(rot_elip)  0
%      0           0         1];

% origin3 = [-1; 2]; r3 = 2*0.5;
% plot_circle(origin3,r3);
plotres = [500,  500,  90,    70];
% Dt = [0.5,      0.5, 0.5, 0.5];
% Dt = [0.01, 0.02, 0.01, 0.01];

plot_color = {'-sr','-pb','-vg','-+k'};
i = 1;
for j = 1:length(X0)
    % for j = 4:4
    % dt = Dt(j);
    dt = 0.05;
    x0 = X0(j); y0 = Y0(j); th0 = Th0(j);
    kplot = 0; c = 0;
    
    %     x0 = -0.5; y0 = 3.5; th0 = 5*pi/4; kplot = 0; c = 0;
    x = x0; y = y0; th = th0;
    t = 0;
    ne = inf; v = 0; w = 0;
    
    %   % k = 2; % OK
    k = 4; % OK
    XX = [];
    % YY = [];
    % V = []; 
    % W = [];
    % while ne > 0.01 && t < 300
    while t < 300
        kplot = kplot+1;
        % robot model
        x = x + v*cos(th)*dt;
        y = y + v*sin(th)*dt;
        th = th + w*dt;
        
        X = [x; y];
        % beta
        beta_arena = r_arena^2 - (X-origin_arena)'*(X-origin_arena);
        beta1 = (X-origin1)'*(X-origin1) - r1^2;
        beta2 = (X-origin2)'*(X-origin2) - r2^2;
        % beta3 = (X-origin3)'*(X-origin3) - r3^2;
        
        % ellipsoid
        % origin_ellipsoid
        beta3 = [(x-origin_ellipsoid(1))/a   (y-origin_ellipsoid(2))/b]*[(x-origin_ellipsoid(1))/a; (y-origin_ellipsoid(2))/b] - 1;
        
        beta = beta_arena*beta1*beta2*beta3;
        
        betadot_arena = -2*(X-origin_arena);
        betadot1 = 2*(X-origin1);
        betadot2 = 2*(X-origin2);
        % betadot3 = 2*(X-origin3);
        betadot3 = 2*[(x-origin_ellipsoid(1))/a;   (y-origin_ellipsoid(2))/b];
        betadot = betadot_arena*beta1*beta2*beta3 + beta_arena*betadot1*beta2*beta3 + beta_arena*beta1*betadot2*beta3 + beta_arena*beta1*beta2*betadot3(1:2);
        % betadot = betadot_arena*beta1*beta2*beta3 + beta_arena*betadot1*beta2*beta3 + beta_arena*beta1*betadot2*beta3 + beta_arena*beta1*beta2*betadot3;
        
        num1 = 2*X*((X'*X)^k + beta)^(1/k);
        num2 = ( 1/k )*( ((X'*X)^(k) + beta)^(-1+(1/k)) )*( (k*(X'*X)^(k-1))*2*X+betadot )*(X'*X);
        num = num1 - num2;
        den = ((X'*X)^k + beta).^(2/k);
        grad_phi = num/den;
        p = norm(grad_phi);
        alpha = myAtan2(grad_phi(1),grad_phi(2));
        q = sin(th-alpha);
        
        % control law
        v = kr*p;
        w = kq*q;
        
        % error
        % X = [x; y; th];
        ne = norm(X(1:2));
        
        t = t + dt;
        
        % plot
        plot(x,y,'.r','LineWidth',2);
        grid on; hold on;
        
        if (rem(kplot,plotres(j)) == 0 || kplot == 1) && c < 3
            % if (rem(10*(round(ne)-1),2) == 0 || kplot == 1) && c < 5
            % draw_robot(x,y,th); grid on; hold on;
            draw_robot_tb3(x,y,th,kplot); grid on; hold on;
            c = c+1;
        end
        
        axis([-4.5 1.5 -1.5 4.5]);
        
        
        %         % Movie
        %         Mov(i) = getframe;
        %         i = i + 1
        XX = [XX; t x];
        % YY = [YY; t y];
        % V = [V; t v];
        % W = [W; t w];
    end
    
%     plot(XX(:,1),XX(:,2),plot_color{j}); grid on; hold on; drawnow
%     xlabel('t (s)'); ylabel('x (m)');
%     legend('Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4');
    % axis([0 300 -3.5 0.5]);
    
%     plot(YY(:,1),YY(:,2),plot_color{j}); grid on; hold on; drawnow
%     xlabel('t (s)'); ylabel('y (m)');
%     legend('Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4');
%     % axis([0 50 0 3.5]);
    
%     plot(V(:,1),V(:,2),plot_color{j}); grid on; hold on; drawnow
%     xlabel('t (s)'); ylabel('v (m/s)');
%     legend('Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4');
%     axis([0 300 0 1.8]);
    
%     plot(W(:,1),W(:,2),plot_color{j}); grid on; hold on; drawnow
%     xlabel('t (s)'); ylabel('w (rad/s)');
%     legend('Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4');
%     % axis([0 50 -5 3]);
    
end
% rgb = [46,139,87];
% plot(0,0,'*','Color',rgb/255,'LineWidth',2); hold on
% axis([-4.5 1.5 -1.5 4.5]);
% drawnow

% Mov(i) = getframe;
%
% video = VideoWriter('ObstacleAvoidance.mp4','MPEG-4');
% open(video)
% writeVideo(video,Mov);
% close(video)