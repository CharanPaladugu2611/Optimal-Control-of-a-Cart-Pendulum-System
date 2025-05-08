clear variables;
clc;

% System parameters
Mass = 1000;
Mass1 = 100;
Mass2 = 100;
length_of_pendulum_1 = 20;
length_of_pendulum_2 = 10;

% Controller parameters
Q = [26  0  0  0  0  0;
     0  45  0  0  0  0;
     0   0 24 0  0  0;
     0   0  0  14  0  0;
     0   0  0  0 26 0;
     0   0  0  0  0  16];
R = 0.005;

% System matrices
A_val = [0 1 0 0 0 0;
         0 0 -(Mass1*9.81)/Mass 0 -(Mass2*9.81)/Mass 0;
         0 0 0 1 0 0;
         0 0 -((Mass+Mass1)*9.81/(Mass*length_of_pendulum_1)) 0 -(Mass2*9.81)/(Mass*length_of_pendulum_1) 0;
         0 0 0 0 0 1;
         0 0 -(Mass1*9.81)/(Mass*length_of_pendulum_2) 0 -((Mass+Mass2)*9.81/(Mass*length_of_pendulum_2)) 0];
B_val = [0; 1/Mass; 0; 1/(Mass*length_of_pendulum_1); 0; 1/(Mass*length_of_pendulum_2)];

% LQR Controller
[K, ~, ~] = lqr(A_val, B_val, Q, R);

% Controllability matrices
CRTBs = {[1 0 0 0 0 0], [0 0 1 0 0 0; 0 0 0 0 1 0], [1 0 0 0 0 0; 0 0 0 0 1 0], [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]};

% Observer poles
L_poles = [-7; -8.5; -1; -2.5; -6; -9.5];

% Initial conditions
x0 = [0; 0; 0.5; 0; 0.6; 0; 0; 0; 0; 0; 0; 0];

% Loop over observers
for i = 1:length(CRTBs)
    CRTB = CRTBs{i};
    
    % Place observer poles
    L = place(A_val', CRTB', L_poles);
    
    % Luenberger matrices
    A_CRTB = [(A_val - B_val*K) B_val*K; zeros(size(A_val)) (A_val - (L'*CRTB))];
    B_CRTB = [B_val; B_val];
    C_CRTB = [CRTB zeros(size(CRTB))];
    D = 0;
    
    % Create Luenberger observer system
    sys_ob = ss(A_CRTB, B_CRTB, C_CRTB, D);
    
    % Display observer gains
    disp(['The Luenberger observer of the system for CRTB', num2str(i), ':']);
    disp(L);
    
    % Display system response to initial conditions
    disp(['System response to initial conditions for CRTB', num2str(i), ':']);
    initial(sys_ob, x0);
    
    % Display system response to unit step input
    disp(['System response to unit step input for CRTB', num2str(i), ':']);
    step(sys_ob);
end