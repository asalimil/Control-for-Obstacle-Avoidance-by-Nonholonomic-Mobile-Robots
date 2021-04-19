function robot_plot(x,y,theta,dt)
% clc
% close all
% clear all
%% =========== Set the paramters =======
% T=0.01; % Sampling Time
% k=2; % Sampling counter
% x=0; % initilize the state x
% y=0; % initilize the state y
% theta=0; % initilize the state theta
tfinal=100; % final simulation time
t=0; % intilize the time 
%=====================================
%% =========== The main loop ==========
while(t<=tfinal) 
t=t+T; % increase the time
% V = 0.75; W = 1; 
theta = W*dt + theta; % calculating theta
x = V*cos(theta)*dt+x; % calculating x
y = V*sin(theta)*dt+y; % calculating y
draw_robot(); % Draw the robot and it's path
k=k+1; % increase the sampling counter
end
%=====================================
%=====================================
end
