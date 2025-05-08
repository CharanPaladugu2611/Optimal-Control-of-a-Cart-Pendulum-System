clear variables;
clc;

% Define symbolic variables
syms Mass_1 Mass_2 Mass Gravity l1 l2 real

% Define system matrices
a = [0 1 0 0 0 0;
     0 0 -Gravity*Mass_1/Mass 0 -Gravity*Mass_2/Mass 0;
     0 0 0 1 0 0;
     0 0 -(Mass*Gravity + Mass_1*Gravity)/(Mass*l1) 0 -Mass_2*Gravity/(Mass*l1) 0;
     0 0 0 0 0 1;
     0 0 -Mass_1*Gravity/(Mass*l2) 0 -(Mass*Gravity + Mass_2*Gravity)/(Mass*l2) 0];

b = transpose([0 1/Mass 0 1/(l1*Mass) 0 1/(l2*Mass)]);
% Define controllabilty matrix using system matrices
controllability_matrix = [b a*b (a^2)*b (a^3)*b (a^4)*b (a^5)*b];
% Controllabilty matrix should be full rank, that is, 6.
rank = rank(controllability_matrix)
% To find the coniditions at which the system is controllable 
DETERMINANT_A = simplify(det(controllability_matrix))
DETERMINANT_B = det(controllability_matrix)
% Check controllability
if rank == size(a,1)
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end