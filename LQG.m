%% System Parameter Declaration
% Cart and pendulum masses, lengths, and gravitational constant
syms Mass Mass1 Mass2 length_of_pendulum_1 length_of_pendulum_2 g;

% System parameter values
Mass = 1000;
Mass1 = 100;
Mass2 = 100;
length_of_pendulum_1 = 20;
length_of_pendulum_2 = 10;

% State-space matrices A and B
A_val = [0 1 0 0 0 0;
         0 0 -(Mass1*9.81)/Mass 0 -(Mass2*9.81)/Mass 0;
         0 0 0 1 0 0;
         0 0 -((Mass+Mass1)*9.81/(Mass*length_of_pendulum_1)) 0 -(Mass2*9.81)/(Mass*length_of_pendulum_1) 0;
         0 0 0 0 0 1;
         0 0 -(Mass1*9.81)/(Mass*length_of_pendulum_2) 0 -((Mass+Mass2)*9.81/(Mass*length_of_pendulum_2)) 0];

B_val = [0; 1/Mass; 0; 1/(Mass*length_of_pendulum_1); 0; 1/(Mass*length_of_pendulum_2)];

% LQR Controller Weighting Matrices
Q = [26 0 0 0 0 0;
    0 0 0 0 0 0; 
    0 0 14 0 0 0;
    0 0 0 42 0 0; 
    0 0 0 0 26 0;
    0 0 0 0 0 75];

R = 0.005;

% LQR Controller Gain Calculation
[K, R_soln, poles] = lqr(A_val, B_val, Q, R);

% Controllability Matrix for State x(t)
C1 = [1 0 0 0 0 0];

% Process and Measurement Noise Matrices
Vd = 0.2 * eye(6);
Vn = 1;

% Observer Gains Calculation for State x(t)
L1 = lqr(A_val', C1', Vd, Vn);

% Augmented Matrices for Luenberger Observer
A_c1 = [(A_val - B_val * K) B_val * K; zeros(size(A_val)) (A_val - L1' * C1)];
B_c1 = [B_val; B_val];
C_c1 = [C1 zeros(size(C1))];

% Initial Conditions for Simulation
x0_lqg = [10; 0; 0.5; 0; 0.6; 0];

% Simulation Time Span
t_span = 0:0.01:600;

% Non-linear System Simulation with LQR Controller and Observer
[ts, x_dots] = ode45(@(t, x) non_lin_sys(t, x, -K * x, L1, C1), t_span, x0_lqg);

% Plotting x(t) for the non-linear system
plot(ts, x_dots(:, 1))
grid
xlabel('Time in seconds')
ylabel('Output')
title('Change of x(t) with respect to time')

% Plotting other states for the non-linear system
for state_idx = 2:6
    plot(ts, x_dots(:, state_idx))
    grid
    xlabel('Time in seconds')
    ylabel('Output')
    title(['Change of state ', num2str(state_idx), ' with respect to time'])
end

%%
function x_dot = non_lin_sys(t, X, F, L, C)
    % Non-linear system dynamics function
    x_dot = zeros(6, 1);
    Mass = 1000;
    Mass1 = 100;
    Mass2 = 100;
    length_of_pendulum_1 = 20;
    length_of_pendulum_2 = 10;
    g_val = 9.81;
    
    % Extracting states from the state vector
    x = X(1);
    x_d = X(2);
    theta1 = X(3);
    theta1_d = X(4);
    theta2 = X(5);
    theta2_d = X(6);
    
    % Observer Output
    obs = L * (x - C * X);
    
    % State dynamics
    x_dot(1) = x_d + obs(1);
    x_dot(2) = (F - ((Mass1 * sin(theta1) * cos(theta1)) + (Mass2 * sin(theta2) * cos(theta2))) * g_val ...
               - (length_of_pendulum_1 * Mass1 * (x_dot(3)^2) * sin(theta1)) ...
               - (length_of_pendulum_2 * Mass2 * (x_dot(5)^2) * sin(theta2))) / ...
               (Mass1 + Mass2 + Mass - (Mass1 * (cos(theta1)^2)) - (Mass2 * (cos(theta2)^2))) + obs(2);
    x_dot(3) = theta1_d + obs(3);
    x_dot(4) = ((cos(theta1) * x_dot(2) - g_val * sin(theta1)) / length_of_pendulum_1) + obs(4);
    x_dot(5) = theta2_d + obs(5);
    x_dot(6) = (cos(theta2) * x_dot(2) - g_val * sin(theta2)) / length_of_pendulum_2 + obs(6);
end