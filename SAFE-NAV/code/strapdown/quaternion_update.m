function [q_update] = quaternion_update (w_nb_b,q,dt)

% description: this fuction computes the updated quaternion

% INPUTS:
% 1- w_nb_b: turn rate of the b-frame with repsect to the n-frame in the
% b-frame [rad/s]
% 2- q: previous quaternion
% 3- dt: sampling time

% OUTPUT:
% q_update: updated quaternion
    
w_nb_b_abs = norm(w_nb_b);

if w_nb_b_abs == 0,
    
    q_update = q;
else
    
    c=cos(0.5*w_nb_b_abs*dt);
    s=sin(0.5*w_nb_b_abs*dt);
    
    c1 = (w_nb_b(1)*s)/w_nb_b_abs;
    c2 = (w_nb_b(2)*s)/w_nb_b_abs;
    c3 = (w_nb_b(3)*s)/w_nb_b_abs;
    c4=c;
    
    
    A=[ c4  c3 -c2 c1;
        -c3  c4  c1 c2;
         c2 -c1  c4 c3;
        -c1 -c2 -c3 c4];
    
    q_update = A * q;
    q_update   = q_update / norm(q);
end

end