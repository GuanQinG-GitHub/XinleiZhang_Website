clear
clc
close all;

%% define variables
N = 2; % state dimension 
dt = 0.01; % temporarily discretization

T = 10;
K = floor(T/dt) +1;

% Define A (control bounds)
A = -1:0.2:1;

% define cost functions
% optimal control problem: sum_{k=0}^{K-1} L(Xk, ak) + g(XK) 
L = @(x1, x2, a) (x1).^2; % stage cost
g = @(x1 , x2) (x1).^2 + (x2).^2; % terminal cost

% define dynamic equations
% x1' = x1 + dt* x2; x2' = x2 + dt*a; 
next_x1 = @(x1, x2, a) x1 + dt*x2;
next_x2 = @(x1, x2, a) x2 + dt*a;

%% Dynaimc programming
% step 1. V(x,K) = g(x)
% step 2. V(x,k) = min_{a \in A} L(x,a) + V(f(x,a),K+1) 
% discretize x space = R^n and save all V data.

% discretize x space, x1 \in [-10, 10], x2 \in [-5,5]
grid_min = [-10; -5];
grid_max = [10; 5];
% # of grid points at each axis
N_grid = [41; 41]; 
[X1_grid, X2_grid] = ...
    ndgrid( linspace(grid_min(1),grid_max(1),N_grid(1)), ...
            linspace(grid_min(2),grid_max(2),N_grid(2))  );
% linspace(grid_min(1),grid_max(1),N_grid(1)): N-grid(1) dimensional
% vector between grid_min and grid_max
% alternative: meshgrid(x,y)

Timer_start = tic;

% Backward computation: value function and Q function
% step 1. V(x,K) = g(x)
V(:,:,K) = g(X1_grid, X2_grid);
% step 2. V(x,k) = min_{a \in A} L(x,a) + V(f(x,a),k+1) 
for k = K-1:-1:1
    % define an interpolation function V(:,:,k+1)
    VNext_interpolation_Func = griddedInterpolant(X1_grid, X2_grid, V(:,:,k+1));
    % Note: if you want to find V(0.5,0.4,k+1) = VNext_interpolation_Func(0.5,0.4)
    % Q-value: Q(x1,x2,a,k) = L(x,a) + V(f(x,a),K+1) 
    for aa = 1:length(A) % Q(x1,x2,k){aa}
        % compute the next state 
        NextState1 = next_x1(X1_grid, X2_grid, A(aa));
        NextState2 = next_x2(X1_grid, X2_grid, A(aa));

        % Q(x1,x2,a,k) = L(x1, x2, a) + V(f(x,a),K+1) 
        Q(:,:,aa,k) = L(X1_grid, X2_grid, A(aa)).*dt + ...
            VNext_interpolation_Func( NextState1, NextState2 );
    end
    % V(:,:,k) = % min_{a \in A} L(x,a) + V(f(x,a),K+1) = min_a Q(x1,x2,a,k)
    % compute V value from Q functions
    V(:,:,k) = min( Q(:,:,:,k) ,[], 3);
end

% Forward computation: optimal control + trajectory
XTraj(:,1) = [2;0.4];
for k = 1:K-1
    % find optimal control from Q-functions
    % V(x1,x2,k) = min_a Q(x1,x2,a,k): a is a minimizer of the Q functions
    % approximate Q(x1,x2, 1, k)
    tmp_Q_value = [];
    for aa = 1:length(A)
        Q_Func{aa} = griddedInterpolant(X1_grid, X2_grid, Q(:,:,aa,k));
        tmp_Q_value = [tmp_Q_value, Q_Func{aa}(XTraj(1,k),XTraj(2,k))];
    end
    [min_value, min_idx] = min(tmp_Q_value);

    OptCtrl(k) = A(min_idx);
    % find optimal trajectory
    XTraj(1, k+1) = next_x1( XTraj(1,k), XTraj(2,k), OptCtrl(k) );
    XTraj(2, k+1) = next_x2( XTraj(1,k), XTraj(2,k), OptCtrl(k) );
end

End_time = toc(Timer_start);

%% display the computation time
disp("time cost: " + num2str(End_time) + " sec");

%% Plot V
figure(1);
for k = K:-100:1
    surf(X1_grid,X2_grid,V(:,:,k))
    title(['V at ', num2str( (k-1)*dt )]);
    axis([-10 10 -5 5 0 3200]);
    colorbar;
    xlabel('position'); ylabel('velocity');
    set(gca,'FontSize', 20');
    pause(0.1);      
end


% final cost calculation
cost_OptCtrl = 0;
for i = 1:K-1
    cost_OptCtrl = cost_OptCtrl + L(XTraj(1,i), XTraj(2,i), OptCtrl(i)).*dt;
end
cost_OptCtrl = cost_OptCtrl + g(XTraj(1,end), XTraj(2,end));

%% Plot the final optimal trajectory in x1-x2 space
figure(2); hold on; grid on; box on;
plot(XTraj(1,:), XTraj(2,:),'LineWidth',1);
scatter(XTraj(1,1),XTraj(2,1),40,"red","LineWidth",1);
scatter(0,0,40,"k","*","LineWidth",1);
legend("State trajectory","Start state","Target state");
xlabel("X(1)");ylabel("X(2)");

%% Plot the state trajectories over time
t_sequence = 0:dt:T;
figure(3);sgtitle('State Trajectory over time');
subplot(2,1,1);
grid on; box on; hold on;
p1 = plot(t_sequence, XTraj(1,:),'LineWidth',1);
xlabel('t');ylabel('X(1)');ylim([-10,10]);
subplot(2,1,2)
grid on; box on; hold on;
p2 = plot(t_sequence, XTraj(2,:),'LineWidth',1);
xlabel('t');ylabel('X(2)');ylim([-5,5]);

%% Plot the control sequance
control_sequence = OptCtrl;
t_sequence = 0:dt:T;
figure(4); hold on; grid on;box on;
plot(t_sequence(1:end-1),control_sequence,'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',4);
line([0,T-dt],[1,1],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,T-dt],[-1,-1],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-1.5,1.5]);
title('Optimal Control Squence');
xlabel('t');ylabel('Control Input');
legend('\alpha','Control boundaries','');

%% Animation of the movement of double integrator
% Animate the movement along a 1D axis
t_sequence = 0:dt:T;

v = VideoWriter('Animation of double integrator','MPEG-4');
v.Quality = 100;
v.FrameRate = 100;

figure(5); hold on; box on; grid on;
open(v);

set(gcf, 'Color', 'white');
set(gcf, 'Position', [500, 500, 600, 200]); % [left, bottom, width, height]

% Set axis limits
axis([-3 3 -0.2 0.2]);
xlabel('Position (m)');
set(gca, 'YTick', []); % Remove y-axis ticks

p0 = scatter(0, 0, 80, 'blue');

% Initialize plots
p1 = plot(NaN, NaN, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
q1 = quiver(NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.2, 'Color', "#CC0000");
q2 = quiver(NaN, NaN, NaN, NaN, 0, 'g', 'LineWidth', 0.8, 'MaxHeadSize', 0.2, 'Color', "#00CC00");

% Update legend once
legend([p0, p1, q1, q2], {'Target position', 'X(1)-position', 'X(2)-velocity', 'Ctrl-accel'});

for i = 1:length(t_sequence)    
    % Update position
    set(p1, 'XData', XTraj(1, i), 'YData', 0);
    
    % Update velocity arrow
    set(q1, 'XData', XTraj(1, i), 'YData', 0, 'UData', XTraj(2, i), 'VData', 0);
    
    % Update acceleration arrow
    if i < length(t_sequence)
        set(q2, 'XData', XTraj(1, i), 'YData', 0, 'UData', OptCtrl(i), 'VData', 0);
    else
        set(q2, 'XData', XTraj(1, i), 'YData', 0, 'UData', 0, 'VData', 0);
    end
    
    % timer in title
    title(sprintf('Time: %.2f s', t_sequence(i)));
    
    % pause
    pause(dt);
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);