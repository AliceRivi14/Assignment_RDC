%% MATLAB SOLUTION - ASSIGNMENT 1: STATIC EQUILIBRIUM OF SERIAL MANIPULATORS

% This code shows how to solve the two exercises proposed in the assignment 1 of Robot Dynamics & Control.

% When you run the program, you will first be asked 5 times which point of exercise 1 you want to perform
% (e.g. 1.1, 1.2, up to 1.5) and then you will be asked another 5 times which point of exercise 2 you want to
% perform (e.g. 2.1, 2.2, up to 2.5).

%% Exercise 1 - RR robot

Assembly_DataFile;
% I consider the absolute reference system 0 coincident with the reference system of the 1st link

% m1 = mass 1st link + mass 1st motor
% m2 = mass 2nd link + mass 2nd motor
m1 = smiData.Solid(1).mass/2.204622476 + smiData.Solid(3).mass/2.204622476; % [kg = lbm/2.204622476]
m2 = smiData.Solid(2).mass/2.204622476 + smiData.Solid(3).mass/2.204622476; % [kg] 

% Lenght of the 2 links
l1 = 1;         % [m]
l2 = 1;         % [m]

% Centre of mass of the link-motor system in the local reference frame
C1 = (((smiData.Solid(1).mass/2.204622476)*smiData.Solid(1).CoM*10^-3) + ((smiData.Solid(3).mass/2.204622476)*smiData.Solid(3).CoM*10^-3))/m1; % [m]
C2 = (((smiData.Solid(2).mass/2.204622476)*smiData.Solid(2).CoM*10^-3) + ((smiData.Solid(3).mass/2.204622476)*smiData.Solid(3).CoM*10^-3))/m2; % [m]
% Points calculate wrt the local reference system
P1 = [1 0 0];           % [m]
P2 = C2 + [-0.2 0 0];   % [m]
P3 = C1 + [0.4 0 0];    % [m]

g = [0 -9.81  0]';      % [m/s^2]
z = [0 0 1]';

for i = 1:5
prompt = "\nExercise 1 \n" + "Which exercise do you want to solve? " ;
ex = input(prompt);

    if ex == 1.1
        th1 = pi/2;
        th2 = - pi/2;
    elseif ex == 1.2
         th1 = 0;
         th2 = pi/2;
    elseif ex == 1.3
         th1 = pi/6;
         th2 = pi/3;
    elseif ex == 1.4
         th1 = pi/6;
         th2 = pi/3;
    elseif ex == 1.5
         th1 = pi/6;
         th2 = -pi/3;
    end

    % Transformation matrix of the 1st link with respect to absolute reference system
    T0_1 = [ cos(th1) -sin(th1)   0        0;
             sin(th1)  cos(th1)   0        0;  
                0       0         1        0;
                0       0         0        1];


    % Transformation matrix of the 2nd link with respect to reference system 1
    T1_2 = [ cos(th2) -sin(th2)   0        1;
             sin(th2)  cos(th2)   0        0;  
               0       0         1         0;
               0       0         0         1];

   % Rotation matrix of the 1st link with respect to absolute reference system
   R0_1 = T0_1(1:3,1:3);
   % Rotation matrix of the 2nd link with respect to reference system 1
   R1_2 = T1_2(1:3,1:3);

    
    % Distance between point and reference system
    C1_0 = T0_1*[C1 1]';             % distance C1-<0>
    C2_0 = T0_1*T1_2*[C2 1]';        % distance C2-<0>
    C2_2 = R0_1*R1_2*C2';            % distance C2-<2>
    P1_0 = T0_1*T1_2*[P1 1]';        % distance P1-<0>
    P1_2 = R0_1*R1_2*P1';            % distance P1-<2>
    P2_0 = T0_1*T1_2*[P2 1]';        % distance P2-<0>
    P2_2 = R0_1*R1_2*P2';            % distance P2-<2>
    P3_0 = R0_1*P3';                 % distance P3-<0>
    
    C1_0(4,:) = [];
    C2_0(4,:) = [];
    P1_0(4,:) = [];
    P2_0(4,:) = [];
    
    % Jacobian J = [Ja1    Ja2
    %               Jl1    Jl2]
    J_C1 = [      z            [0 0 0]';
            cross(z,C1_0)     [0 0 0]'];

    J_C2 = [      z                z        ;
            cross(z,C2_0)   cross(z,C2_2)];

    J_P1 = [      z                z        ; 
            cross(z,P1_0)   cross(z,P1_2)];

    J_P2 = [      z                z        ;
            cross(z,P2_0)   cross(z,P2_2)];

    J_P3 = [      z            [0 0 0]';
            cross(z,P3_0)     [0 0 0]'];

    % Tau = - [Tau1
    %          Tau2]
    if ex == 1.1
        Fext_1 = m1*g;      % [N]
        Fext_2 = m2*g;      % [N]

        Tau_1 = - (J_C1'*[[0 0 0]' ; Fext_1] + J_C2'*[[0 0 0]' ; Fext_2])

    elseif ex == 1.2
        Fext_1 = m1*g;      % [N]
        Fext_2 = m2*g;      % [N]

        Tau_2 = - (J_C1'*[[0 0 0]' ; Fext_1] + J_C2'*[[0 0 0]' ; Fext_2])

    elseif ex == 1.3
         Fext = [-0.7 -0.5  0]';        % [N]

         Tau_31 = - (J_P1'*[[0 0 0]' ; Fext])   % Equilibrium torques when Fext acts on P1
         
         Tau_32 = - (J_P2'*[[0 0 0]' ; Fext])   % Equilibrium torques when Fext acts on P2

    elseif ex == 1.4
         Fext_3 = [1.5 -0.3 0]';        % [N]
         Mext_1 = 1.2;                  % [Nm]

         Tau_4 = - (J_P1'*[[0 0 Mext_1]' ; [0 0 0]'] + J_P3'*[[0 0 0]' ; Fext_3])

    elseif ex == 1.5
         Fext_1 = m1*g;                 % [N]
         Fext_2 = m2*g;                 % [N]
         Fext_P2 = [1.2 -0.2 0]';       % [N]
         Fext_P3 = [-0.4 1.2 0]';       % [N]

         Tau_5 = - (J_C1'*[[0 0 0]' ; Fext_1] + J_C2'*[[0 0 0]' ; Fext_2] + J_P2'*[[0 0 0]' ; Fext_P2] + J_P3'*[[0 0 0]' ; Fext_P3])

    end
end

%% Exercise 2 - PR robot

Assembly_DataFile;
% I consider the absolute reference system coincident with the reference system of the 2nd link

% m2 = mass 2nd link + mass motor
m2 = smiData.Solid(2).mass/2.204622476 + smiData.Solid(3).mass/2.204622476; % [kg = lbm/2.204622476]

% Lenght of the link
l2 = 1;     % [m]

% Centre of mass of the link-motor system in the reference system 
C2 = (((smiData.Solid(2).mass/2.204622476)*smiData.Solid(2).CoM*10^-3) + ((smiData.Solid(3).mass/2.204622476)*smiData.Solid(3).CoM*10^-3))/m2; % [m]
% Points calculate wrt the local reference system
P1 = [1 0 0];             % [m]
P2 = C2 + [0.15 0 0];     % [m]
P3 = [-0.2 0 0];          % [m]

x = [1 0 0]';
z = [0 0 1]';
g = [0 -9.81  0]';        % [m/s^2]

for i = 1:5
prompt = "\nExercise 2 \n" + "Which exercise do you want to solve? " ;
ex = input(prompt);
    if ex == 2.1
        th2 = pi/4 - pi/2;
    elseif ex == 2.2
         th2 = pi/2 - pi/2;
    elseif ex == 2.3
         th2 = pi/4 - pi/2;
    elseif ex == 2.4
         th2 = pi/4 - pi/2;
    elseif ex == 2.5
         th2 = pi/4 - pi/2;
    end

    % Rotation matrix of the 2nd link with respect to absolute reference system
    R0_2 = [ cos(th2) sin(th2)   0;
            -sin(th2) cos(th2)   0;  
                0       0        1];
    
    % Distance between Pi and frame 0
    C2_0 = R0_2*C2';    % distance C2-<0>
    P1_0 = R0_2*P1';    % distance P1-<0>
    P2_0 = R0_2*P2';    % distance P2-<0>
    P3_0 = P3;          % distance P3-<0>
    
    % Jacobian J = [Ja1    Ja2
    %               Jl1    Jl2]

    J_C2 = [[0 0 0]'        z       ;
               x      cross(z,C2_0)];
    
    J_P1 = [[0 0 0]'        z       ; 
               x      cross(z,P1_0)];
    
    J_P2 = [[0 0 0]'        z      ;
               x     cross(z,P2_0)];
    
    J_P3 = [[0 0 0]'    [0 0 0]';
               x        [0 0 0]'];
    
    %Tau = - [ f1 ;
    %         Tau2]

    if ex == 2.1
        Fext = m2*g;        % [N]

        Tau_1 = - J_C2'*[[0 0 0]' ; m2*g]

    elseif ex == 2.2
         Fext = m2*g;       % [N]

         Tau_2 = - J_C2'*[[0 0 0]' ; m2*g]

    elseif ex == 2.3
         Fext_1 = [-0.8 -0.8  0]';      % [N]

         Tau_3 = - (J_P1'*[[0 0 0]' ; Fext_1])

    elseif ex == 2.4
         Fext_2 = [-0.8 -0.2 0]';       % [N]
         Mext_1 = 0.5;                  % [Nm]

         Tau_4 = - (J_P1'*[[0 0 Mext_1]'; [0 0 0]'] + J_P2'*[[0 0 0]' ; Fext_2])

    elseif ex == 2.5
         Fext_2 = m2*g;                 % [N]
         Fext_P1 = [0.5 -0.6 0]';       % [N]
         Fext_P2 = [1.0 -0.4 0]';       % [N]

         Tau_5 = - (J_C2'*[[0 0 0]' ; Fext_2] + J_P1'*[[0 0 0]' ; Fext_P1] + J_P2'*[[0 0 0]' ; Fext_P2])

    end
end

