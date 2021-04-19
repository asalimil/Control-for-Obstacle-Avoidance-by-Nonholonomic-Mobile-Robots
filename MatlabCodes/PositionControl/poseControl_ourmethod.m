clc; clear; close all;

x0 = -1; y0 = -1; th0 = 0*pi/2;

if 0 < th0 && th0 <= pi
    th0 = th0 + 2*pi;
elseif -pi < th0 && th0 <= 0
    th0 = th0 - 2*pi;
end

% X0 = [x0; y0; th0];
x = x0; y = y0; th = th0;
t = 0; dt = 0.1; ne = inf; 
v = 0; w = 0;
kr = -0.2; kf = -0.1; ka = 0.6;
% kr = 0.2; kf = -0.1; ka = 0.6;
flag = 0;

while ne > 0.001
    % robot model
    x = x + v*cos(th)*dt;
    y = y + v*sin(th)*dt;
    th = th + w*dt;
    p = sqrt(x^2 + y^2);
    % alpha with atan2
    % alpha = -th + atan2(y,x);
    % alpha with myAtan2
    beta = myAtan2(x,y);
    alpha = -th + beta;
%     if flag == 0
%         alpha0 = alpha;
%         flag = 1;
%     end
    phi = pi/2 - th;
    
    % control law
    v = kr*p;
    w = ka*alpha + kf*phi;
   
    % error
    X = [x; y; th];
    ne = norm(X(1:2));
    
    t = t + dt;
       
    % plot
    % disp(['x: ', num2str(x), ', y: ', num2str(y)])
    plot(x,y,'.r','LineWidth',1); grid on; hold on
    % plot(t,alpha*180/pi,'*b'); grid on; hold on
    % plot(t,th*180/pi,'*k'); grid on; hold on
    % plot(t,phi*180/pi,'*g'); grid on; hold on
    % plot(t,beta*180/pi,'*g'); grid on; hold on
    axis([-2.5 2.5 -2.5 2.5])
    % plot(t,x,'*r','LineWidth',3); grid on; hold on;
    % plot(t,y,'*b','LineWidth',3); grid on; hold on;
    drawnow;
end






