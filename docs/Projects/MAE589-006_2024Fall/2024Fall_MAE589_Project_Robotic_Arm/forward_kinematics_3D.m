function end_position = forward_kinematics_3D(joint_angles)
%FORWARD_KINEMATICS Summary of this function goes here
%   args:
%   joint_angles: joint angles of the robot arm in a Nx1 column vector (radius)
%   end_position: cartesian position/translation vector of the end-effector
%   parameters:
%   L1 - L6: length of links in puma560
L1 = 0; L2 = -0.2337; L3 = 0.4318; L4 = 0.0203; L5 = 0.0837; L6 = 0.4318;

if length(joint_angles) ~= 6
    disp("input dimension error");
else
    % coordinate transform to Peter Corke's definition
    joint_offset = [0,-pi/2,pi/2,0,0,0]; % offset in the initial-configuration definition
    for i = 1:length(joint_angles)
        joint_angles(i) = joint_angles(i) + joint_offset(i);
    end
    joint_angles(2) = -joint_angles(2);
    joint_angles(3) = -joint_angles(3);
    joint_angles(5) = -joint_angles(5);
    % multiplication of elementary transformation
    T_end = T_transl([0;0;L1]) * T_rotz(joint_angles(1)) * T_roty(joint_angles(2))...
            * T_transl([0;L2;0]) * T_transl([0;0;L3]) * T_roty(joint_angles(3))...
            * T_transl([L4;0;0]) * T_transl([0;L5;0]) * T_transl([0;0;L6])...
            * T_rotz(joint_angles(4)) * T_roty(joint_angles(5)) * T_rotz(joint_angles(6));
    
    end_position = T_end(1:3,4); % extract the position part
end

end

