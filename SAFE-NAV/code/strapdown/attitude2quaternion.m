 function [q] = attitude2quaternion (phi,theta,psi)

% description: this fuction computes the direct cosine matrix as a function
% of the attitude angles: phi, theta, psi.

% INPUTS:
% 1- phi : roll angle [rad]
% 2- theta : pitch angle [rad]
% 3- psi : yaw angle [rad]

% OUTPUT:
% DCMbn : 4*1 quaternion vector

a = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
b = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
c = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
d = cos(phi/2)*cos(theta/2)*sin(psi/2) + sin(phi/2)*sin(theta/2)*cos(psi/2);

q = [b; c; d; a;];

end