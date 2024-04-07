function [w_nb_b] = Compu_w_nb_b (w_ie_n,w_en_n,wb,DCMnb)

% description: this fuction computes the turn rate of the b-frame with repsect to the n-frame in the b-frame 

% INPUTS:
% 1- w_ie_n: turn rate of earth in the n-frame [rad/s]
% 2- w_en_n: turn rate of the n-frame with respect to the earth in the n-frame [rad/s]
% 3- w_b: measured turn rate [rad/s]
% 4- DCMnb: 3*3 direct cosine matric that transfoms from the n-frame to the
% b-frame

% OUTPUT:
% w_nb_b: turn rate of the b-frame with repsect to the n-frame in the
% b-frame [rad/s]
    
w_nb_b = wb - DCMnb*(w_ie_n - w_en_n);    
end