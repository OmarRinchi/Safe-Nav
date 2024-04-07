function [w_en_n] = Compu_w_en_n (lat,vel,h)

% description: this fuction computes the turn rate of the n-frame with repsect to the earth in the n-frame 

% INPUTS:
% 1- lat: latitude [rad]
% 2- vel: vector that has the NED velocity (vn, ve and vd) [m/s]
% 3- h: altitude [m]

% OUTPUT:
% w_en_n: turn rate of the n-frame with repsect to the earth in the n-frame
% [rad/s]
    
    ve = vel(2);
    vn = vel(1);
    h = abs(h);
    
    [R_N,R_E] = earth(lat);

    w_en_n(1,1) = (ve /(R_E + h));                  
    w_en_n(2,1) = (-(vn /(R_N + h)));              
    w_en_n(3,1) = (-(ve * tan(lat) / (R_E + h)));  
    
end