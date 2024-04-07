function [vel_n] = velocity_update(f_e_n,vel,dt)

% description: this fuction computes the updated valocity in the NED frame

% INPUT:
% 1- f_e_n: updated acceleration [m/s^2] 
% 2- previous velocity in the NED frame [m/s]
% 3- sampling time

% OUTPUT:
% vel_n: updated velocity in the NED frame [m/s]

vel_n = vel + (f_e_n' * dt);

end