function [euler] = DCMbn2Attitude (DCMbn)

% description: this fuction converts the DCMbn to attitude angles

% INPUT:
% 1- DCMbn: 3*3 Direct cosine matrix that transfoms from the b-frame to the
% n-frame

% OUTPUT:
% euler: 1*3 vector that contains the attitude angles [phi theta psi] [rad]

phi   = atan2( DCMbn(3,2), DCMbn(3,3) );    % roll
theta = asin (-DCMbn(3,1) );                % pitch
psi   = atan2( DCMbn(2,1), DCMbn(1,1) );    % yaw

euler = [phi theta psi];

end