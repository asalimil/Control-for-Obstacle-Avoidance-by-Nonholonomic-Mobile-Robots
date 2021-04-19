clc; clear; close all;

l_cell = 0.7112;
w_cell = 0.588;
% R = 2*l_cell;
% N = 100;

% th = 0:2*pi/N:2*pi;
% r = 0:R/N:R;
k = 2.5;

origin0 = [-1.5*w_cell; 0*l_cell]; r0 = 2.5*l_cell;
origin1 = [-2*w_cell; 0*l_cell]; ra = l_cell/2; rb = w_cell/2;
origin2 = [-0.5*w_cell; +1*l_cell]; r2 = 0.15;
origin3 = [0*w_cell; -1*l_cell]; r3 = 0.125;

% Phi = []; Xx = [];
% for x = origin0(1)-2*l_cell:0.01:origin0(1)+2*l_cell
%     for y = origin0(2)-2*l_cell:0.01:origin0(2)+2*l_cell
%         X = [x; y];
%         Xx = [Xx X];
%         %
%         beta0 = r0^2 - (X-origin0)'*(X-origin0);
%         beta1 = [(x-origin1(1))/ra   (y-origin1(2))/rb]*[(x-origin1(1))/ra; (y-origin1(2))/rb] - 1;
%         beta2 = (X-origin2)'*(X-origin2) - r2^2;
%         beta3 = (X-origin3)'*(X-origin3) - r3^2;
%         beta = beta0*beta1*beta2*beta3;
%         %
%         phi = (X'*X)/((X'*X)^k + beta)^(1/k);
%         if phi <= 0
%             phi = 0;
%         elseif phi >= 1
%             phi = 1;
%         end
%         Phi = [Phi; phi];
%         % Data = [x; y; Phi];
%         % plot3(x,y,phi,'.b'); hold on; grid on;drawnow
%     end
% end
%
% plot3(Xx(1,:),Xx(2,:),Phi,'.b'); hold on; grid on;drawnow
% axis equal
% % surf(Xx(1,:),Xx(2,:),Phi);
%



% [x,y] = meshgrid(origin0(1)-2*l_cell:0.01:origin0(1)+2*l_cell,origin0(2)-2*l_cell:0.01:origin0(2)+2*l_cell);
[x,y] = meshgrid(origin0(1)-3*l_cell:0.025:origin0(1)+3*l_cell,origin0(2)-3*l_cell:0.025:origin0(2)+3*l_cell);


% for i = 1:285
for i = 1:171
    for j = 1:171
    % for j = 1:285
        beta0 = r0^2 - ((x(i,j)-origin0(1)).^2 + (y(i,j)-origin0(2)).^2);
        beta1 = ((x(i,j)-origin1(1))/ra).^2 + ((y(i,j)-origin1(2))/rb).^2 - 1;
        beta2 = ((x(i,j)-origin2(1)).^2 + (y(i,j)-origin2(2)).^2) - r2^2;
        beta3 = ((x(i,j)-origin3(1)).^2 + (y(i,j)-origin3(2)).^2) - r3^2;
        beta = beta0*beta1*beta2*beta3;
        %
        phi = (x(i,j)^2 + y(i,j)^2)./(((x(i,j)^2 + y(i,j)^2)^k + beta)^(1/k));
        if imag(phi) ~= 0
           phi = 1; 
        end
        if phi <= 0
            Z(i,j) = 0;
        elseif phi >= 1
            Z(i,j) = 1;
        else
            Z(i,j) = phi;
        end
    end
end
surf(x,y,Z)
axis equal
xlabel('x (m)'); ylabel('y (m)'); zlabel('\phi');
