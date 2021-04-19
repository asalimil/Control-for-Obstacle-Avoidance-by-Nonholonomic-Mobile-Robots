clc; clear; close all;

% OK
kr = 1; kq = 5;
% kr = -2; kq = -5;

% kr = 0.1; kq = 0.5;
% kr = 0.1; kq = 0.5;
% k = 0;

% X0 = [2 1 -1 -2 -2 -1 1 2];
% Y0 = [1 2  2  1 -1 -2 -2 -1];
% Th0 = [1 2  2  3 -3 -2 -2 -1]*pi/4;

% arena plot
origin_arena = [-1; 1]; r_arena = 3;
plot_circle(origin_arena,r_arena)
xlabel('x (m)'); ylabel('y (m)');


% plot obstacles
origin1 = [0.5; 1]; r1 = 0.25;
plot_circle(origin1,r1);

origin2 = [-2; 1]; r2 = 0.5;
plot_circle(origin2,r2);

origin3 = [-1; 3]; r3 = 0.5;
plot_circle(origin3,r3);

origin4 = [-1; -1]; r4 = 0.25;
plot_circle(origin4,r4);

% for j = 1:length(X0)

%     x0 = X0(j); y0 = Y0(j); th0 = Th0(j);
%     k = 0; c = 0;

x0 = -3; y0 = 2; th0 = 1*pi/4; kplot = 0; c = 0;
x = x0; y = y0; th = th0; t = 0; 
dt = 0.5; 
ne = inf; v = 0; w = 0;
k = 2; % OK

while ne > 0.01
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
    beta3 = (X-origin3)'*(X-origin3) - r3^2;
    beta4 = (X-origin4)'*(X-origin4) - r4^2;
    
    beta = beta_arena*beta1*beta2*beta3*beta4;

    betadot_arena = -2*(X-origin_arena);
    betadot1 = 2*(X-origin1);
    betadot2 = 2*(X-origin2);
    betadot3 = 2*(X-origin3);
    betadot4 = 2*(X-origin4);
    betadot = betadot_arena*beta1*beta2*beta3*beta4 + beta_arena*betadot1*beta2*beta3*beta4 + beta_arena*beta1*betadot2*beta3*beta4 + beta_arena*beta1*beta2*betadot3*beta4 + beta_arena*beta1*beta2*beta3*betadot4;
    
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
    ne = norm(X(1:2))
    
    t = t + dt;
    
    % plot
    plot(x,y,'.r','LineWidth',2);
    grid on; hold on;
    
    if (rem(kplot,50) == 0 || kplot == 1) && c < 10
        draw_robot(x,y,th); grid on; hold on;
        c = c+1;
    end
    axis([-4 2 -2 4]);
    drawnow
end


% end



