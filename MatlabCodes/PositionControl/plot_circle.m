function plot_circle(O,R,rgb)


t=0.:0.005:2*pi;
circle = [];

for t = 1:length(t)
    circle = [circle; O(1)+R*cos(t) O(2)+R*sin(t)];
end

plot(circle(:,1), circle(:,2),'.','Color',rgb./255,'LineWidth',3); hold on;
drawnow;


end