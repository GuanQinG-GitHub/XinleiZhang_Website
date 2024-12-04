clear;
close all;
clc;

%% load data
distance_multiple_shooting = load("Results\distance_multiple_shooting.mat");distance_multiple_shooting = distance_multiple_shooting.distance;
distance_mpc = load("Results\distance_MPC_dt_0.2_N_10.mat");distance_mpc = distance_mpc.distance;

iter_time_mpc = load("Results\iteration_time_mpc_dt_0.2_N_10.mat");iter_time_mpc = iter_time_mpc.iter_time;
optimization_time_multiple_shooting = load("Results\optimization_time_multiple_shooting.mat");optimization_time_multiple_shooting = optimization_time_multiple_shooting.time_multiple_shooting;
dt = 0.2;
N_multiple_shooting = width(distance_multiple_shooting) - 1;
N_mpc = width(distance_mpc) - 1;

x_mpc = load("Results\x_mpc_dt_0.2_N_10.mat");x_mpc = x_mpc.x_mpc;
x_multiple_shooting = load("Results\x_multiple_shooting.mat");x_multiple_shooting = x_multiple_shooting.end_position;

Xf = 0.6;
Yf = 0.6;
Zf = 0.2;

x_target = [Xf;Yf;Zf];

joint_angles_multiple_shooting = load("Results\angle_joints_multiple_shooting.mat");joint_angles_multiple_shooting = joint_angles_multiple_shooting.joint_angles;
joint_angles_mpc = load("Results\angle_joints_mpc_dt_0.2_N_10.mat");joint_angles_mpc = joint_angles_mpc.angle_mpc;

ctrl_multiple_shooting = load("Results\ctrl_multiple_shooting.mat");ctrl_multiple_shooting = ctrl_multiple_shooting.angluar_velocity;
ctrl_mpc = load("Results\ctrl_mpc_dt_0.2_N_10.mat");ctrl_mpc = ctrl_mpc.Ctrl_mpc;


time_sequence_mpc_detailed = zeros(1,2.*width(distance_mpc)-1);
distance_mpc_detailed = time_sequence_mpc_detailed;
for i = 1:2:length(time_sequence_mpc_detailed)-2
    time_sequence_mpc_detailed(i + 1) = time_sequence_mpc_detailed(i) + iter_time_mpc((i+1)/2);
    time_sequence_mpc_detailed(i + 2) = time_sequence_mpc_detailed(i + 1) + dt;
    distance_mpc_detailed(i) = distance_mpc((i+1)/2);
    distance_mpc_detailed(i + 1) = distance_mpc_detailed(i);
    if i == length(time_sequence_mpc_detailed)-2
        distance_mpc_detailed(i + 2) = distance_mpc(((i+1)/2) + 1);
    end
end

time_sequence_multiple_shooting = 0:dt:N_multiple_shooting*dt;
time_sequence_multiple_shooting = time_sequence_multiple_shooting + optimization_time_multiple_shooting;
time_sequence_multiple_shooting_pre = 0:dt:time_sequence_multiple_shooting(1);
time_sequence_multiple_shooting_pre(end) = time_sequence_multiple_shooting(1);
time_sequence_multiple_shooting_post = time_sequence_multiple_shooting(end):dt:time_sequence_mpc_detailed(end);

distance_multiple_shooting_pre = distance_multiple_shooting(1).*ones(1,length(time_sequence_multiple_shooting_pre));
distance_multiple_shooting_post = distance_multiple_shooting(end).*ones(1,length(time_sequence_multiple_shooting_post));

joint_angles_multiple_shooting_pre = joint_angles_multiple_shooting(:,1).*ones(3,length(time_sequence_multiple_shooting_pre));
joint_angles_multiple_shooting_post = joint_angles_multiple_shooting(:,end).*ones(3,length(time_sequence_multiple_shooting_post));

ctrl_multiple_shooting_pre = zeros(3,length(time_sequence_multiple_shooting_pre));
ctrl_multiple_shooting_post = zeros(3,length(time_sequence_multiple_shooting_post));

%% plot the distance over time of multiple shooting
figure;hold on;grid on;box on;
p_multile_shooting = plot(time_sequence_multiple_shooting,distance_multiple_shooting,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,distance_multiple_shooting_pre,'LineWidth',1,'Color','b');
xlabel("Time (s)");ylabel("Distance (m)");
legend(p_multile_shooting,"Distance from the end-effector to target",'AutoUpdate','off');
ylim([0,0.85]);

%% plot the distance over time of mpc (zoomed in with optimization stage)
figure;hold on;grid on;box on;
p_mpc = plot(time_sequence_mpc_detailed,distance_mpc_detailed,'LineWidth',1,'Color','r');
xlabel("Time (s)");ylabel("Distance (m)");
legend(p_mpc,"Distance from the end-effector to target",'AutoUpdate','off');
ylim([0,0.85]);
% xlim([0,11]);

%% Plot the distance over time of two methods
figure;hold on;grid on;box on;
p_multile_shooting = plot(time_sequence_multiple_shooting,distance_multiple_shooting,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,distance_multiple_shooting_pre,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,distance_multiple_shooting_post,'LineWidth',1,'Color','b','LineStyle','--');
p_mpc = plot(time_sequence_mpc_detailed,distance_mpc_detailed,'LineWidth',1,'Color','r');
xlabel("Time (s)");ylabel("Distance (m)");
legend([p_multile_shooting,p_mpc],["Direct Optimization", "Model Predictive Control"],'AutoUpdate','off');
ylim([0,0.85]);

%% plot the distance over time comparison (zoomed in)
figure;hold on;grid on;box on;
p_mpc = plot(time_sequence_mpc_detailed,distance_mpc_detailed,'LineWidth',1,'Color','r','Marker','o','MarkerSize',4);
% xlabel("Time (s)");ylabel("Distance (m)");
% legend(p_mpc,"Distance from the end-effector to target",'AutoUpdate','off');
ylim([0,0.85]);xlim([0,11]);

%% plot the end-effector position over time
x_mpc_detailed = repmat(time_sequence_mpc_detailed,3,1);
for i = 1:2:length(time_sequence_mpc_detailed)-2
    x_mpc_detailed(:,i) = x_mpc(:,(i+1)/2);
    x_mpc_detailed(:,i + 1) = x_mpc_detailed(:,i);
    if i == length(time_sequence_mpc_detailed)-2
        x_mpc_detailed(:,i + 2) = x_mpc(:,((i+1)/2) + 1);
    end
end

x_multiple_shooting_pre = repmat(x_multiple_shooting(:,1),1,length(time_sequence_multiple_shooting_pre));
x_multiple_shooting_post = repmat(x_multiple_shooting(:,end),1,length(time_sequence_multiple_shooting_post));

figure;sgtitle("End-effector position over time");
subplot(3,1,1);hold on;grid on;box on;
p_multiple_shooting_joint_angle_x = plot(time_sequence_multiple_shooting,x_multiple_shooting(1,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,x_multiple_shooting_pre(1,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,x_multiple_shooting_post(1,:),'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_joint_angle_x = plot(time_sequence_mpc_detailed,x_mpc_detailed(1,:),'LineWidth',1,'Color','r');
p_x_target_x = line([0,time_sequence_mpc_detailed(end)+10],[x_target(1),x_target(1)],'LineStyle','--','Color','k','LineWidth',1.5);
xlabel("Time (s)");ylabel('x (m)');xlim([0,120]);
legend([p_multiple_shooting_joint_angle_x,p_mpc_joint_angle_x,p_x_target_x],["Direct Optimization","Model Predictive Control","Target"]);

subplot(3,1,2);hold on;grid on;box on;
p_multiple_shooting_joint_angle_y = plot(time_sequence_multiple_shooting,x_multiple_shooting(2,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,x_multiple_shooting_pre(2,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,x_multiple_shooting_post(2,:),'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_joint_angle_y = plot(time_sequence_mpc_detailed,x_mpc_detailed(2,:),'LineWidth',1,'Color','r');
p_x_target_y = line([0,time_sequence_mpc_detailed(end)+10],[x_target(2),x_target(2)],'LineStyle','--','Color','k','LineWidth',1.5);
xlabel("Time (s)");ylabel('y (m)');xlim([0,120]);
% legend([p_multiple_shooting_ee_y,p_mpc_ee_y,p_x_target_y],["Multiple Shooting","Model Predictive Control","Target"]);


subplot(3,1,3);hold on;grid on;box on;
p_multiple_shooting_joint_angle_z = plot(time_sequence_multiple_shooting,x_multiple_shooting(3,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,x_multiple_shooting_pre(3,:),'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,x_multiple_shooting_post(3,:),'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_ee_z = plot(time_sequence_mpc_detailed,x_mpc_detailed(3,:),'LineWidth',1,'Color','r');
p_x_target_z = line([0,time_sequence_mpc_detailed(end)+10],[x_target(3),x_target(3)],'LineStyle','--','Color','k','LineWidth',1.5);
xlabel("Time (s)");ylabel('z (m)');xlim([0,120]);
% legend([p_multiple_shooting_ee_z,p_mpc_ee_z,p_x_target_z],["Multiple Shooting","Model Predictive Control","Target"]);

%% plot state (joint angle) over time of two methods
joint_angles_mpc_detailed = repmat(time_sequence_mpc_detailed,3,1);
for i = 1:2:length(time_sequence_mpc_detailed)-2
    joint_angles_mpc_detailed(:,i) = joint_angles_mpc(:,(i+1)/2);
    joint_angles_mpc_detailed(:,i + 1) = joint_angles_mpc_detailed(:,i);
    if i == length(time_sequence_mpc_detailed)-2
        joint_angles_mpc_detailed(:,i + 2) = joint_angles_mpc(:,((i+1)/2) + 1);
    end
end

% define the state bounds
x_ub = [160,135,135];
x_lb = [-160,-135,-135];

figure;sgtitle("Joint angles over time");
subplot(3,1,1);hold on;grid on;box on;
p_multiple_shooting_joint_angle_x = plot(time_sequence_multiple_shooting,joint_angles_multiple_shooting(1,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,joint_angles_multiple_shooting_pre(1,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,joint_angles_multiple_shooting_post(1,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_joint_angle_x = plot(time_sequence_mpc_detailed,joint_angles_mpc_detailed(1,:).*180./pi,'LineWidth',1,'Color','r');
line([0,time_sequence_mpc_detailed(end)],[x_ub(1),x_ub(1)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[x_lb(1),x_lb(1)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('X(1) (degree)');xlim([0,120]);
legend([p_multiple_shooting_joint_angle_x,p_mpc_joint_angle_x],["Direct Optimization","Model Predictive Control"],"AutoUpdate","off");

subplot(3,1,2);hold on;grid on;box on;
p_multiple_shooting_joint_angle_y = plot(time_sequence_multiple_shooting,joint_angles_multiple_shooting(2,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,joint_angles_multiple_shooting_pre(2,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,joint_angles_multiple_shooting_post(2,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_joint_angle_y = plot(time_sequence_mpc_detailed,joint_angles_mpc_detailed(2,:).*180./pi,'LineWidth',1,'Color','r');
line([0,time_sequence_mpc_detailed(end)],[x_ub(2),x_ub(2)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[x_lb(2),x_lb(2)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('X(2) (degree)');xlim([0,120]);



subplot(3,1,3);hold on;grid on;box on;
p_multiple_shooting_joint_angle_z = plot(time_sequence_multiple_shooting,joint_angles_multiple_shooting(3,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,joint_angles_multiple_shooting_pre(3,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,joint_angles_multiple_shooting_post(3,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');
p_mpc_joint_angle_z = plot(time_sequence_mpc_detailed,joint_angles_mpc_detailed(3,:).*180./pi,'LineWidth',1,'Color','r');
line([0,time_sequence_mpc_detailed(end)],[x_ub(3),x_ub(3)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[x_lb(3),x_lb(3)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('X(3) (degree)');xlim([0,120]);ylim([x_lb(3)-10,x_ub(3)+10]);


%% plot ctrl (angular velocity) over time of two methods
ctrl_mpc_detailed = zeros(3,length(time_sequence_mpc_detailed));
for i = 1:2:length(time_sequence_mpc_detailed)-2
    ctrl_mpc_detailed(:,i) = 0;
    ctrl_mpc_detailed(:,i + 1) = ctrl_mpc(:,(i+1)/2);
end

U_ub = [82,74,122];
U_lb = -U_ub;

figure;sgtitle("Control sequence over time of two methods");
subplot(3,1,1);hold on;grid on;box on;
p_mpc_ctrl_x = plot(time_sequence_mpc_detailed(2:2:end),ctrl_mpc_detailed(1,2:2:end).*180./pi,'LineWidth',1,'Color','r','Marker','.','MarkerSize',5);
p_multiple_shooting_ctrl_x = plot(time_sequence_multiple_shooting(2:end),ctrl_multiple_shooting(1,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,ctrl_multiple_shooting_pre(1,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,ctrl_multiple_shooting_post(1,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');

line([0,time_sequence_mpc_detailed(end)],[U_ub(1),U_ub(1)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[U_lb(1),U_lb(1)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('\alpha(1) (degree/s)');xlim([0,120]);
legend([p_multiple_shooting_ctrl_x,p_mpc_ctrl_x],["Direct Optimization","Model Predictive Control"],"AutoUpdate","off");

subplot(3,1,2);hold on;grid on;box on;
p_mpc_ctrl_y = plot(time_sequence_mpc_detailed(2:2:end),ctrl_mpc_detailed(2,2:2:end).*180./pi,'LineWidth',1,'Color','r','Marker','.','MarkerSize',5);
p_multiple_shooting_ctrl_y = plot(time_sequence_multiple_shooting(2:end),ctrl_multiple_shooting(2,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,ctrl_multiple_shooting_pre(2,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,ctrl_multiple_shooting_post(2,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');

line([0,time_sequence_mpc_detailed(end)],[U_ub(2),U_ub(2)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[U_lb(2),U_lb(2)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('\alpha(2) (degree/s)');xlim([0,120]);



subplot(3,1,3);hold on;grid on;box on;
p_mpc_ctrl_z = plot(time_sequence_mpc_detailed(2:2:end),ctrl_mpc_detailed(3,2:2:end).*180./pi,'LineWidth',1,'Color','r','Marker','.','MarkerSize',5);
p_multiple_shooting_ctrl_z = plot(time_sequence_multiple_shooting(2:end),ctrl_multiple_shooting(3,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_pre,ctrl_multiple_shooting_pre(3,:).*180./pi,'LineWidth',1,'Color','b');
plot(time_sequence_multiple_shooting_post,ctrl_multiple_shooting_post(3,:).*180./pi,'LineWidth',1,'Color','b','LineStyle','--');

line([0,time_sequence_mpc_detailed(end)],[U_ub(3),U_ub(3)],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,time_sequence_mpc_detailed(end)],[U_lb(3),U_lb(3)],'LineStyle','--','Color','k','LineWidth',0.8);
xlabel("Time (s)");ylabel('\alpha(3) (degree/s)');xlim([0,120]);ylim([U_lb(3)-10,U_ub(3)+10]);