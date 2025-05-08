%% Defining variables
syms Mass1 g Mass2 Mass Length_of_pendulum_1 Length_of_pendulum_2
Mass1 = 1000;
Mass2 = 100;
Mass = 1000;
Length_of_pendulum_1 = 20;
Length_of_pendulum_2 = 10;
Gravity = 9.81;

%% Linearized A and B
A = [0 1 0 0 0 0; 0 0 -Mass1*Gravity/Mass 0 -Mass2*Gravity/Mass 0; 0 0 0 1 0 0; 0 0 -((Mass*Gravity)+(Mass1*Gravity))/(Mass*Length_of_pendulum_1) 0 -Gravity*Mass2/(Mass*Length_of_pendulum_1) 0; 0 0 0 0 0 1; 0 0 -Mass1*Gravity/(Mass*Length_of_pendulum_2) 0 -((Mass*Gravity)+(Mass2*Gravity))/(Mass*Length_of_pendulum_2) 0];
B = [0; 1/Mass; 0; 1/(Length_of_pendulum_1*Mass); 0; 1/(Length_of_pendulum_2*Mass)];
c = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
d = [1;0;0];
Rank = rank([B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B]);

%% LQR Controller
Q = [26 0 0 0 0 0;
    0 0 0 0 0 0; 
    0 0 14 0 0 0;
    0 0 0 42 0 0; 
    0 0 0 0 26 0;
    0 0 0 0 0 75];
R = 0.005;
[K,S,P] = lqr(A,B,Q,R);
sys = ss(A-B*K,B,c,d);
step(sys,200);
eig(A-B*K)
disp(eig(A-B*K))