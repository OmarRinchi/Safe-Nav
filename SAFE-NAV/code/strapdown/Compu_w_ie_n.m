function [w_ie_n] = Compu_w_ie_n (lat)

% description: this fuction computes the turn rate of earth in the n-frame 

% INPUTS:
% lat: latitude [rad]

% OUTPUT:
% w_ie_n: turn rate of earth in the n-frame [rad/s]

w_ie=7.292115e-5;

w_ie_n=[w_ie*cos(lat), 0, -w_ie*sin(lat)]';

end