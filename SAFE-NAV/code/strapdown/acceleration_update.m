function [f_e_n] = acceleration_update(fn,w_ie_n,w_en_n,vel,g_n)

% description: this fuction computes the updated acceleration in the NED frame

% INPUT:
% 1- fn: 3*1 measured acceleration in the n-frame [m/s^2] 
% 2- w_ie_n: turn rate of earth in the n-frame [rad/s]
% 3- w_en_n: turn rate of the n-frame with respect to the earth in the n-frame [rad/s]
% 4- previous velocity in the NED frame [m/s]
% 5- gravity vector in the n-frame [m/s^2]

% OUTPUT:
% f_e_n: updated acceleration in the NED frame [m/s]

f_e_n = fn - skewm(vel) * (w_en_n + 2 * w_ie_n) - g_n;
end