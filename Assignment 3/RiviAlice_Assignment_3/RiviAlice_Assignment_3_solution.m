%% MATLAB SOLUTION - ASSIGNMENT 3: Dynamic Robot Control

% This code shows how to solve the 4 exercises proposed in the assignment 3
% of Robot Dynamics & Control.

% Each exercise is carried out through the implementation of different blocks
% on Simulink.

% Running the code will show the Simulink files and simulations for each exercise.

Robot = loadrobot('universalUR5');
Robot.DataFormat = 'row';
Robot.Gravity = [0 0 -9.81];

%% Exercise 1 - Gravity Compensation

open_system('Ex1_GravityCompensation.slx')
Simulation = sim("Ex1_GravityCompensation.slx");


q0 = Simulation.q.Data(1,:);                                               % Initial joint configuration
Euler = Simulation.EulerXYZ.Data(:,end);                                   % Orientation of the end-effector
Pose = Simulation.PoseXYZ.Data(:,end);                                     % Pose of the end-effector
Tau1 = Simulation.Tau1.Data(1,:);                                          % Gravity compensation

disp('Exercise 1');
fprintf('Initial joint configuration: %d %d %d %d %d %d \n', q0);
fprintf('Gravity compensation: %d %d %d %d %d %d \n\n', Tau1);

%% Exercise 2 - Linear Joint Control

delta_q = [pi/4 -pi/6 pi/4 pi/3 -pi/2 pi];                                 
q_des = q0 + delta_q;                                                      % Desired joint configuration

open_system('Ex2_LinearJointControl.slx')
Simulation = sim("Ex2_LinearJointControl.slx");
Tau2 = Simulation.Tau2.Data(1,:);                                          % Torque command to reach desired joint configuration

disp('Exercise 2')
fprintf('Initial joint configuration: %d %d %d %d %d %d \n', q0);
fprintf('Desired joint configuration: %d %d %d %d %d %d \n', q_des);
fprintf('Torque: %d %d %d %d %d %d \n\n', Tau2);

%% Exercise 3 - Linear Cartesian Control

x0 = [Pose' Euler'];                                                       % Initial cartesian configuration
delta_x = [0.2 -0.08 -0.15 pi/4 -pi/4 pi/2];
x_des = x0 + delta_x;                                                      % Desired cartesian configuration

open_system('Ex3_LinearCartesianControl.slx')
Simulation = sim("Ex3_LinearCartesianControl.slx");
Tau3 = Simulation.Tau3.Data(:,end);                                        % Torque command to reach desired cartesian configuration

disp('Exercise 3')
fprintf('Initial cartesian configuration: %d %d %d %d %d %d \n', x0);
fprintf('Desired cartesian configuration: %d %d %d %d %d %d \n', x_des);
fprintf('Torque: %d %d %d %d %d %d \n\n', Tau3);


%% Exercise 4 -  Computed Torque Control

q0 = q0';                                                                  % Initial joint configuration
q_des = q_des';                                                            % Desired joint configuration

open_system('Ex4_ComputedTorqueControl.slx')
Simulation = sim("Ex4_ComputedTorqueControl.slx");
Tau4 = Simulation.Tau4.Data(:,end);                                        % Torque command to reach desired joint configuration

disp('Exercise 4')
fprintf('Initial joint configuration: %d %d %d %d %d %d \n', q0);
fprintf('Desired joint configuration: %d %d %d %d %d %d \n', q_des);
fprintf('Torque: %d %d %d %d %d %d \n\n', Tau4);
