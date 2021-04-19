clc; clear; close all;

syms kp kq ka kf real

A1 = [kp       0     0
       0      -kp    +kp
       0      -kq    +kq];

eig(A1)

A2 = [-kp       0     0
       0       +kp    -kp
       0       +kq    -kq];
   
eig(A2)

% A3 = [+kp   0   0
%         0  -kp +kp
%        -kq   0   0];
% 
% eig(A3)