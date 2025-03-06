%% Step 1: Define your symbolic DH table
armstrong = Robot;
%define variables
syms theta1 theta2 theta3 theta4 

%define DHTable based upon syms
DHTable = [
    theta1,                     96.326,       0,   -90;  %f2   good
    theta2-atand(128/24),       0,      130.231,    0;  %f3
    theta3+atand(128/24),       0,          124,    0;  %f4
    theta4,                     0,        133.4,    90   %fee
];

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
%create an instance of Robot.

%obtain the homogenous transformation matrix of our instatiated object.
ht_ee = armstrong.dh2fk(DHTable);
%disp
disp('Transformation Matrix for the End Effector');
disp(ht_ee);

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.
matlabFunction(ht_ee, "File", "dhtable", "Vars", [theta1, theta2, theta3, theta4])
% Write the fk_3001 function in Robot.m to complete sign-off #5

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

%% LAB 4 CODE
% jacobian position
translation_vector = [ht_ee(1, 4); ht_ee(2, 4); ht_ee(3, 4)];

jacobian_position = jacobian(translation_vector, [theta1, theta2, theta3, theta4]);

% jacobian velocity
T0_1 = armstrong.dh2fk(DHTable(1:4, :));   %1st joint transformation matrix
T0_2 = armstrong.dh2fk(DHTable(1:3, :));   %2nd joint transformation matrix
T0_3 = armstrong.dh2fk(DHTable(1:2, :));   %3rd joint transformation matrix
T0_4 = armstrong.dh2fk(DHTable(1:1, :));   %4th joint transofmration matrix

% angvel_1 = T0_1(1:3, 3);
angvel_1 = [0; 0; 1];
angvel_2 = T0_2(1:3, 3);
angvel_3 = T0_3(1:3, 3);
angvel_4 = T0_4(1:3, 3);

jacobian_angvel = [angvel_1, angvel_2, angvel_3, angvel_4];

jacobian_matrix = [(180/pi) * jacobian_position; jacobian_angvel];
disp('Jacobian Matrix:');
disp(jacobian_matrix);

% export
matlabFunction(jacobian_matrix, 'File', 'jacob3001_sym.m', 'Vars', [theta1, theta2, theta3, theta4]);

armstrong.jacob3001([12, -13, 45, -80]);