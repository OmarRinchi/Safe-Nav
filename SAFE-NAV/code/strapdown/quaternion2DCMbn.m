function [DCMbn] = quaternion2DCMbn (q)

% description: this fuction computes converts the quaternion to DCMbn

% INPUT:
% 1- q: 4*1 quaternion

% OUTPUT:
% DCMbn: 3*3 Direct cosine matrix that transfoms from the b-frame to the
% n-frame

DCMbn = zeros(3);
a = q(4); b = q(1); c = q(2); d = q(3);

DCMbn(1,1) = a*a + b*b - c*c - d*d;
DCMbn(1,2) = 2*(b*c - a*d);
DCMbn(1,3) = 2*(a*c + b*d);
DCMbn(2,1) = 2*(a*d + b*c);
DCMbn(2,2) = a*a - b*b + c*c - d*d;
DCMbn(2,3) = 2*(c*d - a*b);
DCMbn(3,1) = 2*(b*d - a*c);
DCMbn(3,2) = 2*(c*d + a*b);
DCMbn(3,3) = a*a - b*b - c*c + d*d;

end