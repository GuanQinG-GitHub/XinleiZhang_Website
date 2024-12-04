function T_rz = T_rotz(angle)
%T_ROTX Summary of this function goes here
%   args:
%   angle: joint rotation angle in radius
%   T_rz: transformation matrix of the rotating about z-axis in world frame
T_rz = [cos(angle), -sin(angle), 0, 0;
        sin(angle),  cos(angle), 0, 0;
                 0,           0, 1, 0;
                 0,           0, 0, 1];
end

