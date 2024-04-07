function [pos_n] = position_update(pos, vel, dt)

% description: this fuction computes the updated psition vector [lat lon h]

% INPUT:
% 1- pos: previous position [rad rad m] 
% 2- vel: velocity [m/s]
% 3- sampling time [s]

% OUTPUT:
% vel_n: updated position vector [rad rad dt]

lat = pos(1); 
lon = pos(2); 
h   = pos(3);
vn  = vel(1);
ve  = vel(2);
vd  = vel(3);

h_n  = h - (vd) * dt;

[R_N,R_E] = earth(lat);

lat_up = vn / (R_N + h_n);

lat_n = lat + (lat_up) * dt;

[R_N,R_E] = earth(lat_n);

lon_up  = ve / ((R_N + h) * cos (lat_n));

lon_n = lon + (lon_up) * dt;

pos_n = [lat_n lon_n h_n];

end