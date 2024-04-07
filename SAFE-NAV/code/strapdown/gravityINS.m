function [g_n] = gravityINS(lat, h)

% description: this fuction computes the gravity vector in the n-frame

% INPUT:
% 1- lat: latitude [rad]
% 2- h: altitude [m]

% OUTPUT:
% g_n: the gravity vector in the n-frame [m/s^2]

g0 = 9.780318*(1 + 5.3024e-3*sin(lat)^2 - 5.9e-6*sin(2*lat)^2);

[R_N,R_E] = earth(lat);
Ro = sqrt(R_N .* R_E);
g = (g0 ./ (1 + (h ./ Ro)).^2);
g_n=[0 0 g];

end