clear variables;
clc;
% Define symbolic variables
syms Mass1 Mass2 Mass Gravity L_1 L_2 x t1 t2 dx dt1 dt2 real
% Define system matrices
aT = transpose([0 1 0 0 0 0;
                0 0 -Gravity*Mass1/Mass 0 -Gravity*Mass2/Mass 0;
                0 0 0 1 0 0;
                0 0 -(Mass*Gravity + Mass1*Gravity)/(Mass*L_1) 0 -Mass2*Gravity/(Mass*L_1) 0;
                0 0 0 0 0 1;
                0 0 -Mass1*Gravity/(Mass*L_2) 0 -(Mass*Gravity + Mass2*Gravity)/(Mass*L_2) 0]);
b = transpose([0 1/Mass 0 1/(L_1*Mass) 0 1/(L_2*Mass)]);
% Weight matrices based on the output vector
% Output vectors: x(t)
c1T = transpose([1 0 0 0 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 0 0]);
% Output vectors: theta1(t), theta2(t)
c2T = transpose([0 0 0 0 0 0;
                 0 0 1 0 0 0;
                 0 0 0 0 1 0]);
% Output vectors: x(t), theta2(t)
c3T = transpose([1 0 0 0 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 1 0]);
% Output vectors: x(t), theta1(t), theta2(t)
c4T = transpose([1 0 0 0 0 0;
                 0 0 1 0 0 0;
                 0 0 0 0 1 0]);
obs_matrix_1 = [c1T aT*c1T (aT^2)*c1T (aT^3)*c1T (aT^4)*c1T (aT^5)*c1T]

obs_matrix_2 = [c2T aT*c2T (aT^2)*c2T (aT^3)*c2T (aT^4)*c2T (aT^5)*c2T]
obs_matrix_3 =[c3T aT*c3T (aT^3)*c3T (aT^3)*c3T (aT^4)*c3T (aT^5)*c3T]
obs_matrix_4 =[c4T aT*c4T (aT^4)*c4T (aT^3)*c4T (aT^4)*c4T (aT^5)*c4T]

% Display the matrices and their ranks
disp('obs_matrix_1');
disp('obs_matrix_2');disp('obs_matrix_3');disp('obs_matrix_4');
% Find rank of the observability for each of the set of output vectors
% Running will directly give out the ranks in the command window
rank1 = rank([c1T aT*c1T (aT^2)*c1T (aT^3)*c1T (aT^4)*c1T (aT^5)*c1T])
if rank1 == size(obs_matrix_1, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

rank2 = rank([c2T aT*c2T (aT^2)*c2T (aT^3)*c2T (aT^4)*c2T (aT^5)*c2T])
if rank2 == size(obs_matrix_2, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
  end
rank3 = rank([c3T aT*c3T (aT^3)*c3T (aT^3)*c3T (aT^4)*c3T (aT^5)*c3T])
 if rank3 == size(obs_matrix_3, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
  end
rank4 = rank([c4T aT*c4T (aT^4)*c4T (aT^3)*c4T (aT^4)*c4T (aT^5)*c4T])
  if rank4 == size(obs_matrix_4, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
  end
 
  
  


