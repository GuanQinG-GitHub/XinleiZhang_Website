function T_rx = T_rotx(angle)
%T_ROTX Summary of this function goes here
%   args:
%   angle: joint rotation angle in radius
%   T_rx: transformation matrix of the rotating about x-axis in world frame
T_rx = [1,          0,           0, 0;
        0, cos(angle), -sin(angle), 0;
        0, sin(angle),  cos(angle), 0;
        0,          0,           0, 1];
end

