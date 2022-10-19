%% MATLAB SOLUTION - ASSIGNMENT 2: Recursive Inverse Dynamics

% This code shows how to solve the 4 exercises proposed in the assignment 2 
% of Robot Dynamics & Control.

% The first exercise is the NewtonEuler() function, which is called each 
% time Tau is to be calculated.
% Every time you run the programme, you are asked which point of the exercise 
% you want to solve.
% The second and third exercises have two points, while the last one only has one.

% The structure of the robot is given by the position of the points belonging
% to links C, P and Q, the masses and inertia matrices of the links, the 
% type and number of manipulator joints and the k parameter of each link

%% Exercise 2 - Inverse Dynamics of 2R robot
clear 

m = [22 19]';           % [Kg]
g_0 = [0 0 0]';         % Null gravity          [m/s^2]
g = [0 -9.81 0]';       % Gravity along Y axis  [m/s^2]
P1 = [0 0 0];           % [m]
P2 = [1 0 0];           % [m]
P3 = [0.8 0 0];         % [m]
C1 = [0.5 0 0];         % [m]
C2 = [0.4 0 0];         % [m]
nJoints = 2;
% 2 rotational joints
Joint = [0 0]';
% Inertial operator with respect to the centre of mass
I = zeros(3,3,2); 
I(:,:,1) = [0    0     0;
            0   0.4    0;
            0    0    0.4]; % [Kg*m^2]
I(:,:,2) = [0    0     0;
            0   0.3    0;
            0    0    0.3]; % [Kg*m^2]

for i = 1:2
    prompt = "\nExercise 2 \n" + "Which exercise do you want to solve? " ;
    ex = input(prompt);
    
    if ex == 2.1
        th1 = 20*pi/180;
        th2 = 40*pi/180;
    elseif ex == 2.2
        th1 = 90*pi/180;
        th2 = 45*pi/180;
    end
    
    % Transformation matrix of the 1st link with respect to absolute reference system
    T0_1 = [ cos(th1) -sin(th1)   0        0;
             sin(th1)  cos(th1)   0        0;  
                0       0         1        0;
                0       0         0        1];
    % Transformation matrix of the 2nd link with respect to reference system 1
    T1_2 = [ cos(th2) -sin(th2)   0        1;
             sin(th2)  cos(th2)   0        0;  
                0       0         1        0;
                0       0         0        1];

    % Rotation matrix of the 1st link with respect to absolute reference system
    R0_1 = T0_1(1:3,1:3);
    % Rotation matrix of the 2nd link with respect to reference system 1
    R1_2 = T1_2(1:3,1:3);

    % Axis of motion calculated with respect to absolute reference system
    k = zeros(3,2);
    k(:,1) = R0_1*[0 0 1]';
    k(:,2) = R0_1*R1_2*[0 0 1]';
        
    % Position of the point with respect to the absolute reference system 0 in homogenous coordinate
    P = zeros(4,nJoints);
    P(:,1) = [P1 1]';
    P(:,2) = T0_1*[P2 1]';
    Q = zeros(4,nJoints);
    Q(:,1) = P(:,2);
    Q(:,2) = T0_1*T1_2*[P3 1]';
    C = zeros(4,nJoints);
    C(:,1) = T0_1*[C1 1]';
    C(:,2) = T0_1*T1_2*[C2 1]';
    
    P(4,:) = [];
    Q(4,:) = [];
    C(4,:) = [];

    % Inertial operator with respect to the centre of mass rotates respect to the robot configuration
    Ic = zeros(3,3,2);
    Ic(:,:,1) = R0_1*I(:,:,1)*R0_1';
    Ic(:,:,2) = R0_1*R1_2*I(:,:,2)*R1_2'*R0_1';

    if ex == 2.1
        q = [20*pi/180  40*pi/180]';
        q_dot = [0.2  0.15]';
        q_2dot = [0.1  0.085]';
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Null gravity
        Tau_0 = NewtonEuler(Robot, q, q_dot, q_2dot, g_0)                  
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Gravity along Y axis
        Tau = NewtonEuler(Robot, q, q_dot, q_2dot, g)                      
    
    elseif ex == 2.2
        q = [90*pi/180  45*pi/180]';
        q_dot = [-0.8  0.35]';
        q_2dot = [-0.4  0.1]';
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Null gravity
        Tau_0 = NewtonEuler(Robot, q, q_dot, q_2dot, g_0)
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Gravity along Y axis
        Tau = NewtonEuler(Robot, q, q_dot, q_2dot, g)
        
    end
end



%% Exercise 3 - Inverse Dynamics of RP
clear

m = [10 6]';            % [Kg]
g_0 = [0 0 0]';         % Null gravity          [m/s^2]
g = [0 -9.81 0]';       % Gravity along Y axis  [m/s^2]
P1 = [0 0 0];           % [m]
P3 = [1.2 0 0];         % [m]
C1 = [1 0 0];           % [m]
C2 = [0.6 0 0];         % [m]
nJoints = 2;
% 1 rotational and 1 prismatic joint
Joint = [0 1]';
% Inertial operator with respect to the centre of mass
I = zeros(3,3,2);
I(:,:,1) = [0    0     0;
            0   0.4    0;
            0    0    0.4]; % [Kg*m^2]
I(:,:,2) = [0    0     0;
            0   0.3    0;
            0    0    0.3]; % [Kg*m^2]

for i = 1:2
    prompt = "\nExercise 3 \n" + "Which exercise do you want to solve? " ;
    ex = input(prompt);
    
    if ex == 3.1
        th1 = 20*pi/180;
        P2 = [1.6 0 0];     % [m]
    elseif ex == 3.2
        th1 = 120*pi/180;
        P2 = [2 0 0];       % [m]
    end
    
    % Transformation matrix of the 1st link with respect to absolute reference system
    T0_1 = [ cos(th1) -sin(th1)   0        0;
             sin(th1)  cos(th1)   0        0;  
                0       0         1        0;
                0       0         0        1];
    % Transformation matrix of the 2nd link with respect to reference system 1
    T1_2 = [ 1  0  0   P2(1);
             0  1  0    0;  
             0  0  1    0;
             0  0  0    1];

    % Rotation matrix of the 1st link with respect to absolute reference system
    R0_1 = T0_1(1:3,1:3);
    R1_2 = T1_2(1:3,1:3);
    
    % Axis of motion calculated with respect to absolute reference system
    k = zeros(3,2);
    k(:,1) = R0_1*[0 0 1]';
    k(:,2) = R0_1*R1_2*[1 0 0]';

    % Position of the point with respect to the absolute reference system 0 in homogenous coordinate
    P = zeros(4,nJoints);
    P(:,1) = [P1 1]';
    P(:,2) = T0_1*[P2 1]';
    Q = zeros(4,nJoints);
    Q(:,1) = T0_1*[2 0 0 1]';
    Q(:,2) = T0_1*T1_2*[P3 1]';
    C = zeros(4,nJoints);
    C(:,1) = T0_1*[C1 1]';
    C(:,2) = T0_1*T1_2*[C2 1]';
    
    P(4,:) = [];
    Q(4,:) = [];
    C(4,:) = [];
    
    % Inertial operator with respect to the centre of mass rotates respet to the robot configuration
    Ic = zeros(3,3,2);
    Ic(:,:,1) = R0_1*I(:,:,1)*R0_1';
    Ic(:,:,2) = R0_1*R1_2*I(:,:,2)*R1_2'*R0_1';

    if ex == 3.1
        q = [20*pi/180  -0.4]';
        q_dot = [0.08  0.03]';
        q_2dot = [0.1  0.01]';
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Null gravity
        Tau_0 = NewtonEuler(Robot, q, q_dot, q_2dot, g_0)
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Gravity along Y axis
        Tau = NewtonEuler(Robot, q, q_dot, q_2dot, g)
    
    elseif ex == 3.2
        q = [120*pi/180  0]';
        q_dot = [-0.4  -0.08]';
        q_2dot = [-0.1  -0.01]';
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Null gravity
        Tau_0 = NewtonEuler(Robot, q, q_dot, q_2dot, g_0)
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Gravity along Y axis
        Tau = NewtonEuler(Robot, q, q_dot, q_2dot, g)
    
    end
end

%% Exercise 4 - Inverse Dynamics of 3R
clear

% The reference system is positioned so that the Y0 axis is incoming and the
% Z0 axis is vertical, so the coordinates of the gravity and rotation axes of
% the second and third links must be changed

m = [20 20 6]';         % [Kg]
g_0 = [0 0 0]';         % Null gravity          [m/s^2]
g = [0 0 -9.81]';       % Gravity along Z axis  [m/s^2]
P1 = [0 0 0];           % [m]
P2 = [1 0 0];           % [m]
P3 = [0.8 0 0];         % [m]
P4 = [0.35 0 0];        % [m]
C1 = [0.5 0 0];         % [m]
C2 = [0.4 0 0];         % [m]
C3 = [0.175 0 0];       % [m]
nJoints = 3;
% 3 rotational joints
Joint = [0 0 0]';
% Inertial operator with respect to the centre of mass
I = zeros(3,3,3);
I(:,:,1) = [0.2  0   0;
             0  0.2  0;
             0   0  0.8]; % [Kg*m^2]
I(:,:,2) = [0.2  0   0;
             0  0.2  0;
             0   0  0.8]; % [Kg*m^2]
I(:,:,3) = [0.08 0   0;
             0  0.08 0;
             0   0  0.1]; % [Kg*m^2]
    
    prompt = "\nExercise 4 \n" + "Which exercise do you want to solve? " ;
    ex = input(prompt);
    
    if ex == 4.1
        th1 = 20*pi/180;
        th2 = 40*pi/180;
        th3 = 10*pi/180;
    end
    
    % Transformation matrix of the 1st link with respect to absolute reference system 0 
    T0_1 = [ cos(th1) -sin(th1)   0        0;
             sin(th1)  cos(th1)   0        0;  
                0       0         1        0;
                0       0         0        1];
    % Transformation matrix of the 2nd link with respect to reference system 1
    % Rotation about the Ox axis (90°)
    T_x = [1    0   0   0;
           0    0  -1   0;
           0    1   0   0;
           0    0   0   1];
    T1_2 = [ cos(th2) -sin(th2)   0        1;
             sin(th2)  cos(th2)   0        0;  
                0       0         1        0;
                0       0         0        1];
    T1_2 = T_x*T1_2;
    % Transformation matrix of the 3rd link with respect to reference system 2
    T2_3 = [ cos(th3) -sin(th3)   0       0.8;
             sin(th3)  cos(th3)   0        0;  
                0       0         1        0;
                0       0         0        1];

    % Rotation matrix of the 1st link with respect to absolute reference system
    R0_1 = T0_1(1:3,1:3);
    % Rotation matrix of the 2nd link with respect to reference system 1
    R1_2 = T1_2(1:3,1:3);
    % Rotation matrix of the 3rd link with respect to reference system 2
    R2_3 = T2_3(1:3,1:3);
    
    % Axis of motion calculated with respect to absolute reference system
    k = zeros(3,3);
    k(:,1) = R0_1*[0 0 1]';
    k(:,2) = R0_1*R1_2*[0 0 1]';
    k(:,3) = R0_1*R1_2*R2_3*[0 0 1]';
    
    % Position of the point with respect to the absolute reference system 0 in homogenous coordinate
    P = zeros(4,nJoints);
    P(:,1) = [P1 1]';
    P(:,2) = T0_1*[P2 1]';
    P(:,3) = T0_1*T1_2*[P3 1]';
    Q = zeros(4,nJoints);
    Q(:,1) = P(:,2);
    Q(:,2) = P(:,3);
    Q(:,3) = T0_1*T1_2*T2_3*[P4 1]';
    C = zeros(4,nJoints);
    C(:,1) = [C1 1]';
    C(:,2) = T0_1*T1_2*[C2 1]';
    C(:,3) = T0_1*T1_2*T2_3*[C3 1]';
    
    P(4,:) = [];
    Q(4,:) = [];
    C(4,:) = [];
    
    % Inertial operator with respect to the centre of mass rotates respet to the robot configuration
    Ic = zeros(3,3,3);
    Ic(:,:,1) = R0_1*I(:,:,1)*R0_1';
    Ic(:,:,2) = R0_1*R1_2*I(:,:,2)*R1_2'*R0_1';
    Ic(:,:,3) = R0_1*R1_2*R2_3*I(:,:,3)*R2_3'*R1_2'*R0_1';
    
    if ex == 4.1
        q = [20*pi/180  40*pi/180 10*pi/180]';
        q_dot = [0.2  0.15 -0.2]';
        q_2dot = [0.1 0.085 0]';
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Null gravity
        Tau_0 = NewtonEuler(Robot, q, q_dot, q_2dot, g_0)
        
        % Robot structure
        Robot = struct('C', C, 'P', P, 'Q', Q, 'm', m, 'Ic', Ic, 'Joint', Joint, 'nJoints', nJoints, 'k', k);
        % Gravity along Z axis
        Tau = NewtonEuler(Robot, q, q_dot, q_2dot, g)

    end

