function [X,Y,Z] = frameChange(t1,t2,t3,t4,t5)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

L1y = 1;
L1z = 1;
L2 = 1;
L3 = 1;
L4 = 1;
L5 = 1;
L6 = 1;

X = L2*cos(t1)*cos(t2)-L1y*sin(t1)+L3*sin(t1)+L4*(cos(t1)*cos(t2)*cos(t3)-cos(t1)*sin(t2)*sin(t3))+L5*(-cos(t4)*sin(t1)+(cos(t1)*cos(t3)*sin(t2)+cos(t1)*cos(t2)*sin(t3))*sin(t4))+L6*(cos(t5)*(cos(t1)*cos(t2)*cos(t3)-cos(t1)*sin(t2)*sin(t3))-(cos(t4)*(cos(t1)*cos(t3)*sin(t2)+cos(t1)*cos(t2)*sin(t3))+sin(t1)*sin(t4))*sin(t5));
Y = L1y*cos(t1)-L3*cos(t1)+L2*cos(t2)*sin(t1)+L4*(cos(t2)*cos(t3)*sin(t1)-sin(t1)*sin(t2)*sin(t3))+L5*(cos(t1)*cos(t4)+(cos(t3)*sin(t1)*sin(t2)+cos(t2)*sin(t1)*sin(t3))*sin(t4))+L6*(cos(t5)*(cos(t2)*cos(t3)*sin(t1)-sin(t1)*sin(t2)*sin(t3))-(cos(t4)*(cos(t3)*sin(t1)*sin(t2)+cos(t2)*sin(t1)*sin(t3))-cos(t1)*sin(t4))*sin(t5));
Z = L1z-L2*sin(t2)+L4*(-cos(t3)*sin(t2)-cos(t2)*sin(t3))+L5*(cos(t2)*cos(t3)-sin(t2)*sin(t3))*sin(t4)+L6*(cos(t5)*(-cos(t3)*sin(t2)-cos(t2)*sin(t3))-cos(t4)*(cos(t2)*cos(t3)-sin(t2)*sin(t3))*sin(t5));
%X=-L1y*sin(t1) + L3*sin(t1) + cos(L2*t1*t2)^2 + L4*(cos(t1*t2*t3)^3 - cos(sin(t1*t2*t3)^2)) + L5*(-cos(sin(t1*t4)) + 2*cos(sin(t1*t2*t3*t4)^2)^2) + L6*(cos(cos(t1*t2*t3)^3-cos(sin(t1*t2*t3)^2))*t5 - sin(sin(t1*t4)^2 + 2*cos(sin(t1*t2*t3*t4))^3)*t5);
% Y = L1y*cos(t1) + L3*cos(t1) + cos(L2)*sin(t1*t2*t3*t4) + L4*(cos(sin(t1*t2*t3))^2 - sin(t1*t2*t3)^3) + L5*(cos(sin(t1*t4)^2) + 2*cos(sin(t1*t2*t3*t4)^3)) + L6*(cos(cos(sin(t1*t2*t3))^2 - sin(t1*t2*t3)^3)*t5 - sin(-cos(sin(t1*t4)) + 2*cos(sin(t1*t2*t3*t4)^2)^2)*t5);
% Z = L1z - L2*sin(t2) - 2*cos(L4)*sin(t2*t3) + L5*sin(cos(t2*t3)^2 - sin(t2*t3)^2)*t4 + L6*(-2*cos(sin(t2*t3*t5))^2);
end