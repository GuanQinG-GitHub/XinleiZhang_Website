function [X,Y,Z] = PUMA_frameChange(t1,t2,t3)
%PUMA transfer matrix function 
%   convert joint angles on PUMA560 (not including the end effector joints)
%   to 3d coordinates

% t1 = joint_angles(1); t2 = joint_angles(2); t3 = joint_angles(3);
L1 = 0;
L2 = -0.2337;
L3 = 0.4318;
L4 = 0.0203;
L5 = 0.0837;
L6 = 0.4318;

% X = L3*cos(t1)*cos(t2)-L2*sin(t1)+L4*(cos(t1)*cos(t2)*cos(t3)-cos(t1)*sin(t2)*sin(t3));
% Y = L2*cos(t1)+L3*cos(t2)*sin(t1)+L4*(cos(t2)*cos(t3)*sin(t1)-sin(t1)*sin(t2)*sin(t3));
% Z = L1-L3*sin(t2)+L4*(-cos(t3)*sin(t2)-cos(t2)*sin(t3))


 X = L4*cos(t1)*cos(t2)+L2*sin(t1)-L5*sin(t1)+L3*cos(t1)*sin(t2)+L6*(cos(t1)*cos(t3)*sin(t2)+cos(t1)*cos(t2)*sin(t3));
 Y = -L2*cos(t1)+L5*cos(t1)+L4*cos(t2)*sin(t1)+L3*sin(t1)*sin(t2)+L6*(cos(t3)*sin(t1)*sin(t2)+cos(t2)*sin(t1)*sin(t3));
 Z = L1+L3*cos(t2)-L4*sin(t2)+L6*(cos(t2)*cos(t3)-sin(t2)*sin(t3));

end