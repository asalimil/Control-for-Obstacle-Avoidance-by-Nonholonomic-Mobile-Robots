function beta = myAtan2(x,y)
n = 0;

if x > 0 && y >= 0
   beta = atan(y/x); 
elseif x < 0 && y >= 0
   beta = atan(y/x) + pi; 
elseif x < 0 && y < 0
   beta = atan(y/x) + pi; 
elseif x > 0 && y < 0
   beta = atan(y/x) + 2*n*pi;
   % beta = atan(y/x);
elseif x == 0 && y > 0 
   beta = pi/2;
elseif x ==0 && y < 0
   beta = 3*pi/2;
end
    
end