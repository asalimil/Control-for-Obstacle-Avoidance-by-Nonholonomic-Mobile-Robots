function plot_ellipsoid(O,a,b,rot)

R = [cos(rot)   -sin(rot)  0
     sin(rot)   +cos(rot)  0
     0           0         1];

X = [];
for th = 0:pi/200:2*pi
    x = a*cos(th);
    y = b*sin(th);
    X = [X , [O(1); O(2); 1] + R*[x; y; 1]];
    % plot(O(1)+ X(1), O(2)+ X(2), '.b', 'LineWidth',1);
end
plot(X(1,:), X(2,:), 'Color', [160,82,45]/255, 'LineWidth',3); hold on;
fill(X(1,:),X(2,:),'y'); hold on;
drawnow;

end