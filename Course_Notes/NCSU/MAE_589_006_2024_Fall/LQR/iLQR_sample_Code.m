%% Clear workspace, command window, and close figures
clear all      % Remove all variables from the workspace
clc            % Clear the command window
close all;     % Close all open figure windows

%% Define constants and simulation parameters
N = 4;          % State dimension (number of state variables)
dt = 0.1;       % Time step (discrete time increment)
T = 10;         % Total simulation time
K = floor(T/dt) + 1; % Total number of time steps (including the initial state)

%% Define the discrete system dynamics using symbolic variables
% Define symbolic variables for states (x1, x2, x3, x4) and control inputs (a1, a2)
syms x1 x2 x3 x4 a1 a2

% Define the discrete time dynamic equations:
% x-position update: x1_new = x1 + dt * x3 * cos(x4)
f1 = x1 + dt .* x3 .* cos(x4);
% y-position update: x2_new = x2 + dt * x3 * sin(x4)
f2 = x2 + dt .* x3 .* sin(x4);
% Velocity update: x3_new = x3 + dt * a1
f3 = x3 + dt .* a1;
% Heading angle update: x4_new = x4 + dt * a2
f4 = x4 + dt .* a2;

% Combine the four dynamic equations into one vector function F
F = [f1; f2; f3; f4];

% Convert the symbolic function F into a MATLAB function for numerical evaluation
F = matlabFunction(F);

%% Define the linearized system matrices (A and B)
% Compute the partial derivatives (Jacobian) of F with respect to state variables
f1_x1 = diff(f1, x1);  f1_x2 = diff(f1, x2);  f1_x3 = diff(f1, x3);  f1_x4 = diff(f1, x4);
f2_x1 = diff(f2, x1);  f2_x2 = diff(f2, x2);  f2_x3 = diff(f2, x3);  f2_x4 = diff(f2, x4);
f3_x1 = diff(f3, x1);  f3_x2 = diff(f3, x2);  f3_x3 = diff(f3, x3);  f3_x4 = diff(f3, x4);
f4_x1 = diff(f4, x1);  f4_x2 = diff(f4, x2);  f4_x3 = diff(f4, x3);  f4_x4 = diff(f4, x4);

% Compute the partial derivatives (Jacobian) of F with respect to control inputs
f1_a1 = diff(f1, a1);  f1_a2 = diff(f1, a2);
f2_a1 = diff(f2, a1);  f2_a2 = diff(f2, a2);
f3_a1 = diff(f3, a1);  f3_a2 = diff(f3, a2);
f4_a1 = diff(f4, a1);  f4_a2 = diff(f4, a2);

% Form the state transition matrix A_s from the partial derivatives with respect to states
A_s = [f1_x1, f1_x2, f1_x3, f1_x4;
       f2_x1, f2_x2, f2_x3, f2_x4;
       f3_x1, f3_x2, f3_x3, f3_x4;
       f4_x1, f4_x2, f4_x3, f4_x4];
% Convert symbolic expression to a MATLAB function for evaluation
A_s = matlabFunction(A_s);

% Form the control input matrix B_s from the partial derivatives with respect to controls
B_s = [f1_a1, f1_a2;
       f2_a1, f2_a2;
       f3_a1, f3_a2;
       f4_a1, f4_a2];
% Since B_s is constant, convert it directly to a numerical matrix
B_s = double(B_s);

%% Define cost parameters for the iLQR problem
% Define the control cost matrix R (penalizes control effort)
R = 0.5 .* [1, 0;
            0, 1];

% Define the state cost matrix Q (penalizes deviation in the first two state components)
Q = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];

% Define the terminal state cost matrix QK (penalizes final state deviations)
QK = 10 .* [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];

% Initialize the cost-to-go matrix P at the final time step with QK
P(:,:,K) = QK;

%% iLQR Algorithm Initialization

% Initialize control sequence 'a' for each time step (2 controls for K steps)
a = -0.1 .* ones(2, K);

% Initialize the state trajectory matrix X_traj (each column is the state at a time step)
X_traj = zeros(N, K);
% Set the initial state (given initial conditions)
X_traj(:,1) = [2; 0.4; 0; 0];

% Compute the nominal trajectory using the initial control sequence
for i = 1:K-1
    % Evaluate the dynamics function F with current control and state to get next state
    X_traj(:,i+1) = F(a(1,i), a(2,i), X_traj(1,i), X_traj(2,i), X_traj(3,i), X_traj(4,i));
end

% Save the initial nominal trajectory for later comparison
Nominal_X_traj = X_traj;

% Augment cost parameters to include an extra dimension for affine terms in dynamics
Q_augmented = [Q, zeros(4,1);  % Extend Q with zeros (for the additional variable)
               zeros(1,4), 0];   % Bottom-right element is zero
QK_augmented = [QK, zeros(4,1); % Similarly extend QK
                zeros(1,4), 0];

% Initialize the augmented cost-to-go matrix at the final time step with QK_augmented
P_k_augmented(:,:,K) = QK_augmented;

%% iLQR Main Loop Setup
threshold = 1e-3;              % Convergence threshold for trajectory change
error = threshold + 1;         % Initialize error above threshold to enter loop
err_log = [];                  % Log of error values per iteration
X_traj_log(:,:,1) = X_traj;    % Log the initial trajectory
iter = 0;                      % Initialize iteration counter
time1 = cputime;               % Record start CPU time

%% iLQR Iterative Optimization Loop
while error > threshold
    error = 0;  % Reset error at the start of each iteration
    
    % BACKWARD PASS: Compute feedback gains and update cost-to-go matrices
    for k = K-1:-1:1
        % Evaluate the linearized dynamics matrices at the current state:
        % A_k: Linearization with respect to state (evaluated at x3 and x4 components)
        A_k = A_s(X_traj(3,k), X_traj(4,k));
        % B_k: Constant control matrix (does not depend on state here)
        B_k = B_s;
        
        % Compute the residual (affine term) in the dynamics by subtracting the linear terms:
        % C_k = f(x,u) - A_k * x - B_k * u
        C_k = F(a(1,k), a(2,k), X_traj(1,k), X_traj(2,k), X_traj(3,k), X_traj(4,k)) ...
              - A_k * X_traj(:,k) - B_k * a(:,k);
          
        % Construct the augmented state matrix to account for affine dynamics:
        % The augmented state adds an extra dimension that is constant (e.g., 1)
        A_k_augmented = [A_k,       C_k;        % Append the affine term as an extra column
                         zeros(1,4), 1];          % The extra row corresponds to the constant term
        
        % Similarly, augment the control matrix by adding zeros for the extra dimension
        B_k_augmented = [B_k; zeros(1,2)];
        
        % Compute the feedback gain (FB_GainK) at time step k using dynamic programming
        % This involves solving a linear quadratic regulator (LQR) type update:
        % FB_GainK = -inv(R + B'PB)*B'P A
        FB_GainK(:,:,k) = -(R + B_k_augmented' * P_k_augmented(:,:,k+1) * B_k_augmented) \ ...
                           (B_k_augmented' * P_k_augmented(:,:,k+1) * A_k_augmented);
                       
        % Update the cost-to-go matrix P using the Bellman equation for the augmented system:
        % P_k = Q + (FB_GainK)' * R * (FB_GainK) + (A + B*FB_GainK)' * P_{k+1} * (A + B*FB_GainK)
        P_k_augmented(:,:,k) = Q_augmented + FB_GainK(:,:,k)' * R * FB_GainK(:,:,k) + ...
                                (A_k_augmented + B_k_augmented * FB_GainK(:,:,k))' * ...
                                P_k_augmented(:,:,k+1) * (A_k_augmented + B_k_augmented * FB_GainK(:,:,k));
    end

    % FORWARD PASS: Update the control sequence and state trajectory
    for k = 1:K-1
        % Compute the optimal control update using the feedback gain:
        % Augment the current state with a constant 1 to account for the affine term
        OptCtrl_LQR(:,k) = FB_GainK(:,:,k) * [X_traj(:,k); 1]; %#ok<SAGROW>
        
        % Save the previous trajectory (for error calculation)
        Nominal_X_traj(:,k+1) = X_traj(:,k+1);
        
        % Simulate the new state trajectory using the updated control:
        % Evaluate the system dynamics F at the new optimal control and current state
        X_traj(:,k+1) = F(OptCtrl_LQR(1,k), OptCtrl_LQR(2,k), ...
                            X_traj(1,k), X_traj(2,k), X_traj(3,k), X_traj(4,k));
                        
        % Accumulate the squared error between the previous and updated state trajectory
        error = error + (Nominal_X_traj(:,k+1) - X_traj(:,k+1))' * (Nominal_X_traj(:,k+1) - X_traj(:,k+1));
    end
    
    % Update the control sequence for the next iteration with the newly computed optimal control
    a = OptCtrl_LQR;
    
    % Increment the iteration counter and log the current error and trajectory
    iter = iter + 1;
    err_log = [err_log; error];
    X_traj_log(:,:,iter+1) = X_traj;
end

time2 = cputime;  % Record end CPU time
disp("iteration: " + num2str(iter));  % Display the number of iterations taken for convergence

%% Display the optimal cost after the last iteration
optimal_cost = 0;
% Accumulate the running cost over the trajectory (state cost and control cost)
for i = 1:K-1
    optimal_cost = optimal_cost + dt .* (X_traj(:,i)' * Q * X_traj(:,i) + OptCtrl_LQR(:,i)' * R * OptCtrl_LQR(:,i));
end
% Add the terminal cost at the final time step
optimal_cost = optimal_cost + X_traj(:,K)' * QK * X_traj(:,K);
disp("optimal cost: " + num2str(optimal_cost));  % Display the optimal cost value


%% animation of state trajectory in x1-x2 space
figure(1);hold on;box on;grid on;
plot(X_traj(1,1), X_traj(2,1),'Marker','o');
plot(0, 0,'Marker','*');
xlim([0,2]);ylim([0,0.4]);
title("Iteration: ");
for i = 1:length(X_traj_log(1,1,:))
    p1 = plot(X_traj_log(1,:,i), X_traj_log(2,:,i),'b');
    title("Iteration: " + num2str(i - 1));
    pause(0.5);
    delete(p1);
end
plot(X_traj(1,:), X_traj(2,:),'b');


%% plot of the iteration process
figure(2);hold on;box on;grid on;
scatter(X_traj(1,1), X_traj(2,1),'Marker','o');
scatter(0, 0,'Marker','*');
plot(X_traj_log(1,:,1), X_traj_log(2,:,1),'Color','#99CCFF','LineWidth',1);
plot(X_traj_log(1,:,2), X_traj_log(2,:,2),'Color','#33FF99','LineWidth',1);
plot(X_traj_log(1,:,3), X_traj_log(2,:,3),'Color','#CCCC00','LineWidth',1);
plot(X_traj_log(1,:,4), X_traj_log(2,:,4),'Color','#CC6600','LineWidth',1);
plot(X_traj_log(1,:,5), X_traj_log(2,:,5),'Color','#994C00','LineWidth',1);
plot(X_traj_log(1,:,6), X_traj_log(2,:,6),'Color','#990000','LineWidth',1);
xlim([-2,2]);ylim([-0.5,3.5]);
title("x1-x2 Trajectory Iteration Process");
legend('Start point', 'Target point', 'Trajectory Initialization', 'x1-x2 Trajectory iteration 1',...
       'x1-x2 Trajectory iteration 2','x1-x2 Trajectory iteration 3','x1-x2 Trajectory iteration 4',...
       'x1-x2 Trajectory iteration 5');

%% display the computation time
disp("computation time: " + num2str(time2 - time1) + "s");
computation_time_iLQR = time2 - time1;
%% Plot the state trajectory over time
t_sequence = 0:dt:T;
figure(3);sgtitle('State Response over Time');
subplot(4,1,1);
grid on; box on; hold on;
line([0,T],[0,0],'LineWidth',3,'Color',[0.2 0.5 0.9 0.5]);
p1 = plot(t_sequence, X_traj(1,:),'LineWidth',1);
legend('Target state','X(1)');
xlabel('t');ylabel('X(1)');ylim([-0.5,2.5]);

subplot(4,1,2)
grid on; box on; hold on;
line([0,T],[0,0],'LineWidth',3,'Color',[0.2 0.5 0.9 0.5]);
p2 = plot(t_sequence, X_traj(2,:),'LineWidth',1);
legend('Target state','X(2)');
xlabel('t');ylabel('X(2)');ylim([-0.5,2.5]);

subplot(4,1,3);
grid on; box on; hold on;
line([0,T],[0,0],'LineWidth',3,'Color',[0.2 0.5 0.9 0.5]);
p3 = plot(t_sequence, X_traj(3,:),'LineWidth',1);
legend('Target state','X(3)');
xlabel('t');ylabel('X(3)');ylim([-1.5,0.5]);

subplot(4,1,4)
grid on; box on; hold on;
line([0,T],[0,0],'LineWidth',3,'Color',[0.2 0.5 0.9 0.5]);
p4 = plot(t_sequence, X_traj(4,:),'LineWidth',1);
legend('Target state','X(4)');
xlabel('t');ylabel('X(4)');ylim([-0.5,1.5]);

%% Plot the control sequance
control_sequence = OptCtrl_LQR;
t_sequence = 0:dt:T;
figure(4);sgtitle('Optimal Control Squence');
subplot(2,1,1); hold on; grid on;box on
plot(t_sequence(1:end-1),control_sequence(1,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',1);
line([0,T-dt],[0.5,0.5],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,T-dt],[-0.5,-0.5],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-1.5,1.5]);
xlabel('t');ylabel('Control Input');
legend('\alpha(1)','Control boundaries','');

subplot(2,1,2); hold on; grid on;box on
plot(t_sequence(1:end-1),control_sequence(2,:),'Color','#CC0000','LineWidth',1,'Marker','o','MarkerSize',1);
line([0,T-dt],[0.5,0.5],'LineStyle','--','Color','k','LineWidth',0.8);
line([0,T-dt],[-0.5,-0.5],'LineStyle','--','Color','k','LineWidth',0.8);
ylim([-1.5,1.5]);
xlabel('t');ylabel('Control Input');
legend('\alpha(2)','Control boundaries','');