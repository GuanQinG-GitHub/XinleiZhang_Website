clear all
clc
close all

import casadi.*
%% Settings
lamb = 5;
% temporal discretization
setDt = .2;
T = 5;
N = (T/setDt)+1;
dt = T/(N-1);

% target position
Xf = 0.6;
Yf = 0.6;
Zf = 0.2;

x_target = [Xf;Yf;Zf];

time1 = cputime;

opti = casadi.Opti(); % initial casadi optimization setting

A = opti.variable(3,N); % joint angles variable

U = opti.variable(3,N-1); % define control seq (angular velocity)

initial_angle = [0;0;0]; % define initial state

x_ini = forward_kinematics_3D([initial_angle;0;0;0]);

% define joint angles update equations
for tt = 1:N-1
    A(1,tt+1) = A(1,tt) + dt*U(1,tt);
    A(2,tt+1) = A(2,tt) + dt*U(2,tt);
    A(3,tt+1) = A(3,tt) + dt*U(3,tt);
end

% define obstacles
obs_p_1 = [0.5;0.4;0.3]; obs_r_1 = 0.05;
obs_p_2 = [0.7;0.1;0.3]; obs_r_2 = 0.05;
obs_p = [obs_p_1,obs_p_2];
obs_r = [obs_r_1,obs_r_2];

% define the cost
cost = 0;
Q = diag([1,1,1]);
R = diag([1,1,1]);
Q_K = 10.*diag([1,1,1]);

% cost integration
for tt = 1:N
    end_p_temp = forward_kinematics_3D([A(1,tt),A(2,tt),A(3,tt),0,0,0]);

    % add constrains to the optimization struct
    if tt > 1
        opti.subject_to(obstacle_region(end_p_temp,obs_p,obs_r) < 0);
    end
    
    % add the culmulative cost
    if tt < N
        % stage cost
        cost = cost + dt*((end_p_temp - x_target)'*Q*(end_p_temp - x_target) + U(:,tt)'*R*U(:,tt));
    else
        % terminal cost
        cost = cost + lamb*(end_p_temp - x_target)'*Q_K*(end_p_temp - x_target);
    end
    
end

opti.minimize(cost)


% define control bounds
U_ub = pi.*[82,74,122]./180;
U_lb = -U_ub;
% add the control constraints
for i = 1:length(U_ub)
    for tt = 1:N-1
        opti.subject_to(U_lb(i) <= U(i,tt) <= U_ub(i));
    end
end

% define the state bounds
x_ub = pi.*[160,135,135]./180;
x_lb = pi.*[-160,-135,-135]./180;
% add the state constraints
for i = 1:length(x_ub)
    for ii = 1:N
        opti.subject_to(x_lb(i) <= A(i,ii) <= x_ub(i));
    end
end


% define initial states
opti.subject_to(A(1,1) == initial_angle(1));
opti.subject_to(A(2,1) == initial_angle(2));
opti.subject_to(A(3,1) == initial_angle(3));

opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

time2 = cputime;
time_multiple_shooting = time2 - time1;
disp("Time cost for optimzation: " + num2str(time_multiple_shooting));
%% save the results from optimization
joint_angles = sol.value(A);
angluar_velocity = sol.value(U);
end_position = zeros(3,width(joint_angles));
for i = 1:width(joint_angles)
    end_position(:,i) = forward_kinematics_3D([joint_angles(:,i);0;0;0]);
end
joint_angle_degree = 180.*joint_angles./pi;
%% Plot Trajectory and States
time = 0:dt:T;
[x1,y1,z1] = sphere;

sX = x1 * obs_r_1;
sY = y1 * obs_r_1;
sZ = z1 * obs_r_1;
figure(1);hold on;grid on;box on;
plot3(end_position(1,:),end_position(2,:),end_position(3,:));% plot x and y positon

plot3(x_ini(1),x_ini(2),x_ini(3), "o");
plot3(Xf,Yf,Zf, "+");

surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','b');
surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','b');
axis equal;
xlabel("x");ylabel("y");zlabel("z");

%% load the puma 560 robot arm from the Robotics Toolbox
mdl_puma560
view_angle = [1.170119635026772e+02,21.647105567627275];
position_fig = [678.3333333333333,327,907.333333333333,681.3333333333333];
figure(2);hold on;grid on;box on;
set(gcf,'Position',position_fig,'color',"white");
p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
p560.plot3d([joint_angles(:,1);zeros(3,1)]','view',view_angle,'alpha',0.5);
xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
p1 = plot3(end_position(1,1),end_position(2,1),end_position(3,1),'Color','#990000','LineWidth',1);
legend([p_ini,p_goal,p_obs],["Initial Point", "Target Point", "Obstacles"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");

for i = 1:width(joint_angles)
    p560.animate([joint_angles(:,i);zeros(3,1)]');
    delete(p1);
    p1 = plot3(end_position(1,1:i),end_position(2,1:i),end_position(3,1:i),'Color','#990000','LineWidth',1);
    pause(0.1);
end

% %% save the video for the above animation
% v = VideoWriter('3D_View_Multiple_Shooting','MPEG-4');
% v.Quality = 100;
% v.FrameRate = 4;
% 
% 
% mdl_puma560
% view_angle = [1.170119635026772e+02,21.647105567627275];
% position_fig = [678.3333333333333,327,907.333333333333,681.3333333333333];
% figure(2);hold on;grid on;box on;
% set(gcf,'Position',position_fig,'color',"white");
% p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
% p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
% p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
% surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
% axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
% p560.plot3d([joint_angles(:,1);zeros(3,1)]','view',view_angle,'alpha',0.5);
% xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
% p1 = plot3(end_position(1,1),end_position(2,1),end_position(3,1),'Color','#990000','LineWidth',1);
% legend([p_ini,p_goal,p_obs],["Initial Point", "Target Point", "Obstacles"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");
% 
% open(v);
% frame = getframe(gcf);
% writeVideo(v,frame);
% 
% tic
% for i = 1:width(joint_angles)
% 
%     p560.animate([joint_angles(:,i);zeros(3,1)]');
%     delete(p1);
%     p1 = plot3(end_position(1,1:i),end_position(2,1:i),end_position(3,1:i),'Color','#990000','LineWidth',1);
%     pause(0.1);
% 
%     frame = getframe(gcf);
%     writeVideo(v,frame);
% 
% end
% toc
% close(v);

%% plot the 3D figure from the top-down view
view_angle_2 = [89.905882353500587,90];
figure(3);hold on;grid on;box on;
set(gcf,'Position',position_fig,'color',"white");
p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
p560.plot3d([joint_angles(:,1);zeros(3,1)]','view',view_angle_2,'alpha',1);
xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
p1 = plot3(end_position(1,1),end_position(2,1),end_position(3,1),'Color','#990000','LineWidth',1);
legend([p_ini,p_goal,p_obs],["Initial Point", "Target Point", "Obstacles"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");

for i = 1:width(joint_angles)
    p560.animate([joint_angles(:,i);zeros(3,1)]');
    delete(p1);
    p1 = plot3(end_position(1,1:i),end_position(2,1:i),end_position(3,1:i),'Color','#990000','LineWidth',1);
    pause(0.01);
end

% %% save the above animiation
% v = VideoWriter('Top_down_View_Multiple_Shooting','MPEG-4');
% v.Quality = 100;
% v.FrameRate = 4;
% 
% mdl_puma560
% position_fig = [678.3333333333333,327,907.333333333333,681.3333333333333];
% view_angle_2 = [89.905882353500587,90];
% figure(3);hold on;grid on;box on;
% set(gcf,'Position',position_fig,'color',"white");
% p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
% p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
% p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
% surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
% axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
% p560.plot3d([joint_angles(:,1);zeros(3,1)]','view',view_angle_2,'alpha',1);
% xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
% p1 = plot3(end_position(1,1),end_position(2,1),end_position(3,1),'Color','#990000','LineWidth',1);
% legend([p_ini,p_goal,p_obs],["Initial Point", "Target Point", "Obstacles"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");
% 
% open(v);
% frame = getframe(gcf);
% writeVideo(v,frame);
% 
% tic
% for i = 1:width(joint_angles)
% 
%     p560.animate([joint_angles(:,i);zeros(3,1)]');
%     delete(p1);
%     p1 = plot3(end_position(1,1:i),end_position(2,1:i),end_position(3,1:i),'Color','#990000','LineWidth',1);
%     pause(0.1);
% 
%     frame = getframe(gcf);
%     writeVideo(v,frame);
% 
% end
% toc
% close(v);

%% plot the mission-distance over time
distance = zeros(1,width(end_position));
for i = 1:width(end_position)
    distance(i) = norm(end_position(:,i) - x_target,2);
end
figure;hold on;grid on;box on;
plot(0:dt:T,distance);
xlabel("Time (s)");ylabel("Distance (m)");
legend("Distance from the end-effector to target");

%% plot the state sequence
figure;sgtitle("State sequence");
subplot(3,1,1);hold on;grid on;box on;
plot(joint_angle_degree(1,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(joint_angle_degree)],[x_ub(1),x_ub(1)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(joint_angle_degree)],[x_lb(1),x_lb(1)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-180,180]);
ylabel('X_{k}(1)');
legend('X_{k}(1)');

subplot(3,1,2);hold on;grid on;box on;
plot(joint_angle_degree(2,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(joint_angle_degree)],[x_ub(2),x_ub(2)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(joint_angle_degree)],[x_lb(2),x_lb(2)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-180,180]);
ylabel('X_{k}(2)');
legend('X_{k}(2)');

subplot(3,1,3);hold on;grid on;box on;
plot(joint_angle_degree(3,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(joint_angle_degree)],[x_ub(3),x_ub(3)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(joint_angle_degree)],[x_lb(3),x_lb(3)].*180./pi,'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-180,180]);
ylabel('X_{k}(3)');
legend('X_{k}(3)');

%% plot the control sequence
figure;sgtitle("Optimal control sequence");
subplot(3,1,1);hold on;grid on;box on;
plot(angluar_velocity(1,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(angluar_velocity)],[U_ub(1),U_ub(1)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(angluar_velocity)],[U_lb(1),U_lb(1)],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-2,2]);
ylabel('\alpha_{k}(1)');
legend('\alpha_{k}(1)');

subplot(3,1,2);hold on;grid on;box on;
plot(angluar_velocity(2,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(angluar_velocity)],[U_ub(2),U_ub(2)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(angluar_velocity)],[U_lb(2),U_lb(2)],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-2,2]);
ylabel('\alpha_{k}(2)');
legend('\alpha_{k}(2)');

subplot(3,1,3);hold on;grid on;box on;
plot(angluar_velocity(3,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,width(angluar_velocity)],[U_ub(3),U_ub(3)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,width(angluar_velocity)],[U_lb(3),U_lb(3)],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-2,2]);
ylabel('\alpha_{k}(3)');
legend('\alpha_{k}(3)');
%%
function g = obstacle_region(x,p,r)
%   args:
%   x: position vector, dim 3x1
%   g: obstacle function value, dim #obstacle x 1
%   p: center position of the obstacle, dim 3x1 or 3x#obstacles
%   r: radius of the obstacle, scalar or 1x#obstacles
g = [];

    if height(x) ~= 3 && height(p) ~= 3
        disp("dimension error");
    else
        for i = 1:width(p)
            g_i = (x(1) - p(1,i)).^2 + (x(2) - p(2,i)).^2 + (x(3) - p(3,i)).^2 - r(i).^2;
            g = [g;-g_i];
        end
    end

end