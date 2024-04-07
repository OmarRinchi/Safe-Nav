function [DCMnb] = attitude2DCM (phi,theta,psi)

% description: this fuction computes the direct cosine matrix as a function
% of the attitude angles: phi, theta, psi.

% INPUTS:
% 1- phi : roll angle [rad]
% 2- theta : pitch angle [rad]
% 3- psi : yaw angle [rad]

% OUTPUT:
% DCMbn : 3x3 Directi cosine matrix that transfoms from the n-frame to the
% b-frame.

C1 = [cos(psi)  sin(psi) 0; ...
        -sin(psi) cos(psi) 0; ...
         0     0   1];
     
  C2 = [cos(theta)  0  -sin(theta); ...
          0   1     0 ; ...
        sin(theta)  0   cos(theta)];
    
  C3 = [1   0    0;   ...
        0  cos(phi) sin(phi); ...
        0 -sin(phi) cos(phi)];  
 
  DCMnb = C3 * (C2 * C1);

end