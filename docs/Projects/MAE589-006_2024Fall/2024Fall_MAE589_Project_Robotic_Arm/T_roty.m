function T_ry = T_roty(angle)
%T_ROTX Summary of this function goes here
%   args:
%   angle: joint rotation angle in radius
%   T_ry: transformation matrix of the rotating about y-axis in world frame
T_ry = [ cos(angle), 0, sin(angle), 0;
                  0, 1,          0, 0;
        -sin(angle), 0, cos(angle), 0;
                  0, 0,          0, 1];
end

