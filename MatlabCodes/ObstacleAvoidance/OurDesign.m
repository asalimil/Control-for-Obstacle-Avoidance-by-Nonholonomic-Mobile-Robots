clc; clear; close all;

x0 = 1; y0 = -2; th0 = 1*pi/4;

% if 0 < th0 && th0 <= pi
%     th0 = th0 + 2*pi;
% elseif -pi < th0 && th0 <= 0
%     th0 = th0 - 2*pi;
% end

% X0 = [x0; y0; th0];
x = x0; y = y0; th = th0; t = 0; dt = 0.1; ne = inf; v = 0; w = 0;

% kr = 0.1; kq = 0.5;
kr = 0.2; kq = -0.1;
% kr = -0.1; kq = -0.5;
% kr = 0.1; kq = 0.5;
% kr = 0.1; kq = 0.5;

while ne > 0.001
    
    % robot model
    x = x + v*cos(th)*dt;
    y = y + v*sin(th)*dt;
    th = th + w*dt;
    
    %
    p = sqrt(x^2 + y^2);
    alpha = myAtan2(x,y);
    q = sin(th-alpha);
    
    % control law
    v = kr*p;
    w = kq*q;
   
    % error
    X = [x; y; th];
    ne = norm(X(1:2));
    
    t = t + dt;
       
    % plot
    % disp(['x: ', num2str(x), ', y: ', num2str(y)])
    plot(x,y,'.r','LineWidth',2); grid on; hold on;
    % line([x  x+0.1*cos(th)], [y  y+0.1*sin(th)],'LineWidth',2); hold on;
    % line([x-10.1*cos(th)  x+0.1*cos(th)], [y-0.1*sin(th)  y+0.1*sin(th)]); hold on;
    % plot(t,alpha*180/pi,'*b'); grid on; hold on
    % plot(t,th*180/pi,'*k'); grid on; hold on
    % plot(t,phi*180/pi,'*g'); grid on; hold on
    % plot(t,(th-alpha)*180/pi,'*g'); grid on; hold on
    % axis([-2.5 2.5 -2.5 2.5])
    % plot(t,x,'*r','LineWidth',3); grid on; hold on;
    % plot(t,y,'*b','LineWidth',3); grid on; hold on;
    % plot(t,v,'.r','LineWidth',1); grid on; hold on
    
    
    
    % V = 0.5*(p^2)*(1-cos(th-alpha))/kr + 0.5*(p^2)*(1-sin(th-alpha))/(kq-kr);
    % V = 0.5*(p^2)*(1-cos(alpha)); % ***
    % V = 0.5*(p^2)*(1-cos(th)); % ***
    % V = 0.5*(p^2)*kr + (kq-kr)*(alpha-cos(alpha))^2; % ***
    % V = 0.5*(p^2)*kr + (kq-kr)*(1-(cos(th-alpha)^2));
    % V = 0.5*(p^2) + p*(kq-kr)*(1-(cos(th-alpha)^2));
    % V = 0.5*(p^2) + 2*(1-(cos(th-alpha)))^2;
    % V = (p*cos(th-alpha))/kr + p/(kq-kr);
    % V = 1-cos(th-alpha);
    % V = 1-cos(th-alpha);
    % V = -(0.5/kr)*(p^2) - p*(1-cos(th-alpha))/(kq-kr);
    % V = p*(1-cos(th-alpha));
    % V = p*(1+cos(th-alpha));
    % V = p*(1+cos(th-alpha));
    % V = (p^2)*(1-cos((th-alpha))); hold on;
    % V = 0.5*(p^2)/kr + 0.5*p^2*(1-cos((th-alpha)))/kq;
    % plot(t,V,'.r','LineWidth',1); grid on; hold on
    % plot(t,1-cos(th-alpha),'.r','LineWidth',1); grid on; hold on
    % plot(t,p,'.r','LineWidth',1); grid on; hold on
    drawnow;
end






