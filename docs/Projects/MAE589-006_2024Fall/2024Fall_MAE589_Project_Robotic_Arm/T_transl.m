function T = T_transl(p)
% args:
% p: cartesian position/translation vector expressed in the world frame
%    dim 3x1
% T: tranformation matrix of the potision/translation vector p, dim 4x4

T = [diag([1,1,1]), p;
        zeros(1,3), 1];
end