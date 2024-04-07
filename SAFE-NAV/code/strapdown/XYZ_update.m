function [XYZ_n] = XYZ_update(vel_n,XYZ,dt)

% description: this fuction computes the updated position in the NED frame

% INPUT:
% 1- vel_n: updated velocity [m/s] 
% 2- XYZ: previous position in the NED frame [m]
% 3- dt: sampling time

% OUTPUT:
% vel_n: updated velocity in the NED frame [m/s]

XYZ_n = XYZ + (vel_n * dt);

end