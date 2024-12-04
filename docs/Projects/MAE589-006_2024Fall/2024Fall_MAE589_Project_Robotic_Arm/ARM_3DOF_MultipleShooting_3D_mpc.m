clear
clc
close all
%% import casadi to solve the optimization problem
import casadi.*
%% temporal discretization
N = 10; % time-horizon windows
dt = 0.2; % time-step

%% dynamical model
f = @(x,u) [u(1);
            u(2);
            u(3)];

%% optimization formulation

% optimization settings
opti = casadi.Opti();   % initial casadi optimization setting

% initial state
initial_angle = [0;0;0]; % define initial state
x_ini = forward_kinematics_3D([initial_angle;0;0;0]);

% target position
Xf = 0.6;
Yf = 0.6;
Zf = 0.2;

x_target = [Xf;Yf;Zf];

% define obstacles
obs_p_1 = [0.5;0.4;0.3]; obs_r_1 = 0.05;
obs_p_2 = [0.7;0.1;0.3]; obs_r_2 = 0.05;
obs_p = [obs_p_1,obs_p_2];
obs_r = [obs_r_1,obs_r_2];

% define control bounds
U_ub = pi.*[82,74,122]./180;
U_lb = -U_ub;

% define the state bounds
x_ub = pi.*[160,135,135]./180;
x_lb = pi.*[-160,-135,-135]./180;

% cost parameters
Q = diag([1,1,1]);
R = diag([1,1,1]);
Q_K = 10.*diag([1,1,1]);

% %% Uncomment the following section if you want to run the MPC process
% 
% % MPC process
% iter = 0;
% OptTraj_log = [];
% iter_time = [];
% time1 = cputime;
% time_last = time1;
% 
% x_update = x_ini;
% dist_resi = norm(x_update - x_target,2);
% dist_resi_temp = dist_resi;
% %
% angle_mpc = initial_angle;
% angle_update = initial_angle;
% x_mpc = x_ini;
% Ctrl_mpc = [];
% 
% while dist_resi >= 0.01
%     iter = iter + 1;
%     angle_ini = angle_update;
%     dist_resi_temp = dist_resi;
% 
%     A = opti.variable(3,N); % joint angles variable
% 
%     U = opti.variable(3,N-1); % define control seq (angular velocity)
% 
% 
%     % cost integration
%     cost = 0;
%     for tt = 1:N
%         end_p_temp = forward_kinematics_3D([A(1,tt),A(2,tt),A(3,tt),0,0,0]);
% 
%         % add constrains to the optimization struct
%         if tt > 1
%             opti.subject_to(obstacle_region(end_p_temp,obs_p,obs_r) < 0);
%         end
% 
%         % add the culmulative cost
%         if tt < N
%             % stage cost
%             cost = cost + dt*((end_p_temp - x_target)'*Q*(end_p_temp - x_target) + U(:,tt)'*R*U(:,tt));
%         else
%             % terminal cost
%             cost = cost + (end_p_temp - x_target)'*Q_K*(end_p_temp - x_target);
%         end
% 
%     end
% 
%     opti.minimize(cost)
% 
%     % Constraint on dynamical process
%     for tt = 1:N-1
%         A_next = A(:,tt) + dt.*f(A(:,tt),U(:,tt));
%         opti.subject_to(A(:,tt+1) == A_next);
%     end
% 
%     % add the control constraints
%     for i = 1:length(U_ub)
%         for tt = 1:N-1
%             opti.subject_to(U_lb(i) <= U(i,tt) <= U_ub(i));
%         end
%     end
% 
%     % add the state constraints
%     for i = 1:length(x_ub)
%         for ii = 1:N
%             opti.subject_to(x_lb(i) <= A(i,ii) <= x_ub(i));
%         end
%     end
%     % initial state
%     opti.subject_to(A(1,1) == angle_ini(1));
%     opti.subject_to(A(2,1) == angle_ini(2));
%     opti.subject_to(A(3,1) == angle_ini(3));
% 
%     solver_opts = struct('max_iter',500);
%     plugin_opts = struct('print_time',false);
%     opti.solver('ipopt',plugin_opts,solver_opts); % set numerical backend
% 
%     sol = opti.solve();   % actual solve
% 
%     % solve the optimization problem
%     OptTraj = sol.value(A);
%     OptTraj_log(:,:,iter) = OptTraj;
%     OptCtrl = sol.value(U);
% 
%     % only update with the first control
%     angle_update = angle_ini + dt*f(angle_ini, OptCtrl(:,1));
%     x_update = forward_kinematics_3D([angle_update;0;0;0]);
% 
%     angle_mpc = [angle_mpc,angle_update];
%     x_mpc = [x_mpc,x_update];
%     Ctrl_mpc = [Ctrl_mpc,OptCtrl(:,1)];
%     % log the computation time for each mpc iteration
%     time2 = cputime;
%     iter_time = [iter_time,time2-time_last];
%     time_last = time2;
%     % cost residence
%     dist_resi = norm(x_update - x_target,2);
% 
%     % terminate the loop if the innovation smaller than the threshold
%     if abs(dist_resi_temp - dist_resi) <= 1e-4
%         disp("Terminate due to convergence");
%         break
%     end
% 
%     disp("iteration number: " + num2str(iter) + ", distance: " + num2str(dist_resi));
%     disp("Control input 1: " + num2str(OptCtrl(1,1)) + ", Control input 2: " + num2str(OptCtrl(2,1)) + ", Control input 3: " + num2str(OptCtrl(3,1)));
%     disp("x: " + num2str(x_update(1)) + ", y: " + num2str(x_update(2)) + ", z: " + num2str(x_update(3)));
% end
% time2 = cputime;
% computation_time_total = time2 - time1;
% disp("Computation time: " + num2str(computation_time_total) + " (second)");
% 
% % total cost
% mpc_cost = 0;
% L = @(x,u) (x - x_target)'*Q*(x - x_target) + u'*R*u;
% for tt = 1:iter
%     mpc_cost = mpc_cost + dt.*L(x_mpc(:,tt),Ctrl_mpc(:,tt));
% end
% mpc_cost = mpc_cost + 100.*(x_mpc(:,end) - x_target)'*(x_mpc(:,end) - x_target);
% disp("mpc cost: " + num2str(mpc_cost));

%% load the results
angle_mpc = load("Results\angle_joints_mpc_dt_0.2_N_10.mat");angle_mpc = angle_mpc.angle_mpc;
x_mpc = load("Results\x_mpc_dt_0.2_N_10.mat");x_mpc = x_mpc.x_mpc;
OptTraj_log = load("Results\Traj_mpc_dt_0.2_N_10.mat");OptTraj_log = OptTraj_log.OptTraj_log;

%% Plot Trajectory and States
[x1,y1,z1] = sphere;

sX = x1 * obs_r_1;
sY = y1 * obs_r_1;
sZ = z1 * obs_r_1;
figure(1);hold on;grid on;box on;
plot3(x_mpc(1,:),x_mpc(2,:),x_mpc(3,:));% plot x and y positon

plot3(x_ini(1),x_ini(2),x_ini(3), "o");
plot3(Xf,Yf,Zf, "+");

surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3));
surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3));
axis equal;
xlabel("x");ylabel("y");zlabel("z");

%% load the puma 560 robot arm from the Robotics Toolbox
mdl_puma560
view_angle = [1.170119635026772e+02,21.647105567627275];
position_fig = [678.3333333333333,327,907.333333333333,681.3333333333333];
Traj_mpc_x = zeros(height(OptTraj_log),width(OptTraj_log),length(OptTraj_log));
for i = 1:length(OptTraj_log)
    for ii = 1:width(OptTraj_log)
        Traj_mpc_x(:,ii,i) = forward_kinematics_3D([OptTraj_log(:,ii,i);0;0;0]);
    end
end

figure(2);hold on;grid on;box on;
set(gcf,'Position',position_fig,'color',"white");
p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
p560.plot3d([angle_mpc(:,1);zeros(3,1)]','view',view_angle,'alpha',0.5);
xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
p1 = plot3(x_mpc(1,1),x_mpc(2,1),x_mpc(3,1),'Color','#990000','LineWidth',1);
p_plot = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1)+2,'LineWidth',1,'Color','#FFB266'); % plotting purposed
p2 = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
legend([p_ini,p_goal,p_obs,p_plot],["Initial Point", "Target Point", "Obstacles","Prediction"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");


for i = 1:width(angle_mpc)
    p560.animate([angle_mpc(:,i);zeros(3,1)]');
    delete([p1,p2]);
    p1 = plot3(x_mpc(1,1:i),x_mpc(2,1:i),x_mpc(3,1:i),'Color','#990000','LineWidth',1);
    if i < width(angle_mpc)
         p2 = plot3(Traj_mpc_x(1,:,i),Traj_mpc_x(2,:,i),Traj_mpc_x(3,:,i),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
    end
    pause(0.1);
end

%% save the video for the above animation
% v = VideoWriter('3D_View_MPC','MPEG-4');
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
% p560.plot3d([angle_mpc(:,1);zeros(3,1)]','view',view_angle,'alpha',0.5);
% xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
% p1 = plot3(x_mpc(1,1),x_mpc(2,1),x_mpc(3,1),'Color','#990000','LineWidth',1);
% p_plot = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1)+2,'LineWidth',1,'Color','#FFB266'); % plotting purposed
% p2 = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
% legend([p_ini,p_goal,p_obs,p_plot],["Initial Point", "Target Point", "Obstacles","Prediction"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");
% 
% open(v);
% frame = getframe(gcf);
% writeVideo(v,frame);
% 
% for i = 1:width(angle_mpc)
%     p560.animate([angle_mpc(:,i);zeros(3,1)]');
%     delete([p1,p2]);
%     p1 = plot3(x_mpc(1,1:i),x_mpc(2,1:i),x_mpc(3,1:i),'Color','#990000','LineWidth',1);
%     if i < width(angle_mpc)
%          p2 = plot3(Traj_mpc_x(1,:,i),Traj_mpc_x(2,:,i),Traj_mpc_x(3,:,i),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
%     end
%     pause(0.1);
% 
%     frame = getframe(gcf);
%     writeVideo(v,frame);
% 
% end
% 
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
p560.plot3d([angle_mpc(:,1);zeros(3,1)]','view',view_angle_2,'alpha',1);
xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
p1 = plot3(x_mpc(1,1),x_mpc(2,1),x_mpc(3,1),'Color','#990000','LineWidth',1);
p_plot = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1)+2,'LineWidth',1,'Color','#FFB266'); % plotting purposed
p2 = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
legend([p_ini,p_goal,p_obs,p_plot],["Initial Point", "Target Point", "Obstacles","Prediction"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");

for i = 1:width(angle_mpc)
    p560.animate([angle_mpc(:,i);zeros(3,1)]');
    delete([p1,p2]);
    p1 = plot3(x_mpc(1,1:i),x_mpc(2,1:i),x_mpc(3,1:i),'Color','#990000','LineWidth',1);
    if i < width(angle_mpc)
         p2 = plot3(Traj_mpc_x(1,:,i),Traj_mpc_x(2,:,i),Traj_mpc_x(3,:,i),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
    end
    pause(0.01);
end

%% save the above animation
% v = VideoWriter('Top_down_View_MPC','MPEG-4');
% v.Quality = 100;
% v.FrameRate = 4;
% 
% view_angle_2 = [89.905882353500587,90];
% figure(3);hold on;grid on;box on;
% set(gcf,'Position',position_fig,'color',"white");
% p_ini = plot3(x_ini(1),x_ini(2),x_ini(3), "o",'LineWidth',1);
% p_goal = plot3(Xf,Yf,Zf, "+",'LineWidth',1);
% p_obs = surf(sX + obs_p_1(1),sY + obs_p_1(2),sZ + obs_p_1(3),'FaceColor','#0000CC');
% surf(sX + obs_p_2(1),sY + obs_p_2(2),sZ + obs_p_2(3),'FaceColor','#0000CC');
% axis equal;xlim([-0.8,1.2]);ylim([-1,1]);zlim([-1,1]);
% p560.plot3d([angle_mpc(:,1);zeros(3,1)]','view',view_angle_2,'alpha',1);
% xlabel("x (m)");ylabel("y (m)");zlabel("z (m)");
% p1 = plot3(x_mpc(1,1),x_mpc(2,1),x_mpc(3,1),'Color','#990000','LineWidth',1);
% p_plot = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1)+2,'LineWidth',1,'Color','#FFB266'); % plotting purposed
% p2 = plot3(Traj_mpc_x(1,:,1),Traj_mpc_x(2,:,1),Traj_mpc_x(3,:,1),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
% legend([p_ini,p_goal,p_obs,p_plot],["Initial Point", "Target Point", "Obstacles","Prediction"],"Position",[0.664305859802848,0.721331316187595,0.096111719605695,0.075832072617247],"AutoUpdate","off");
% 
% open(v);
% frame = getframe(gcf);
% writeVideo(v,frame);
% 
% for i = 1:width(angle_mpc)
%     p560.animate([angle_mpc(:,i);zeros(3,1)]');
%     delete([p1,p2]);
%     p1 = plot3(x_mpc(1,1:i),x_mpc(2,1:i),x_mpc(3,1:i),'Color','#990000','LineWidth',1);
%     if i < width(angle_mpc)
%          p2 = plot3(Traj_mpc_x(1,:,i),Traj_mpc_x(2,:,i),Traj_mpc_x(3,:,i),'LineWidth',1,'Color','#FFB266'); % predict trajectory in each mpc
%     end
%     pause(0.01);
% 
%     frame = getframe(gcf);
%     writeVideo(v,frame);
% end
% 
%  close(v);

%% plot the mission-distance over time
distance = zeros(1,width(x_mpc));
for i = 1:width(x_mpc)
    distance(i) = norm(x_mpc(:,i) - x_target,2);
end
figure;hold on;grid on;box on;
plot(0:dt:(width(x_mpc)-1)*dt,distance);
xlabel("Time (s)");ylabel("Distance (m)");
legend("Distance from the end-effector to target");
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