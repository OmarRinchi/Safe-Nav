function [R_N,R_E] = earth (lat)

% description: this fuction computes the meridian radius of curvature 
% and the transverse radius of curvature of the earth as a function of the latitude  

% INPUTS:
% lat: latitude [rad]

% OUTPUTs:
% 1- R_N: meridian radius of curvature [m]
% 2- R_E: transverse radius of curvature [m]
    
    R = 6378137;
    e = 0.0818191908426;
    
    R_N = (R*(1 - e^2))/(1 - e^2*sin(lat)^2)^(3/2);
    R_E = R/(1-e^2*sin(lat)^2)^(1/2);
    
end