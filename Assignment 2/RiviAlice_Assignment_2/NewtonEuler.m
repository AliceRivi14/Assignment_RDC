%% Exercise 1 - Recursive Newton-Euler Implementation

% This function allows the inverse dynamics joint torque to be calculated.
% As input, the structure of the robot and the variable parameter of each
% joint with its derivatives are considered.

function [Tau] = NewtonEuler(Robot, q, q_dot, q_2dot, g)
 
% Each parameter is calculated with respect to the absolute reference
% system 0

F = zeros(3,1);                                                            % Force                                      [N]
M = zeros(3,1);                                                            % Monet                                      [Nm]
w = zeros(3,Robot.nJoints);                                                % Angular velocity                           [rad/s]
v = zeros(3,Robot.nJoints);                                                % Linear velocity                            [m/s]
w_dot = zeros(3,Robot.nJoints);                                            % Angular acceleration                       [rad/s^2]
v_dot = zeros(3,Robot.nJoints);                                            % Linear acceleration                        [m/s^2]
v_C = zeros(3,Robot.nJoints);                                              % Linear velocity of the centre of mass      [m/s]
v_Cdot = zeros(3,Robot.nJoints);                                           % Linear acceleration of the centre of mass  [m/s^2]
Tau = zeros(Robot.nJoints,1);                                              % Inverse dynamic joint torques              [Nm]    RJ
                                                                           %                                            [N]     TJ 


% Forward computation
for i = 1:1:Robot.nJoints

    % Rotational joints
    if Robot.Joint(i) == 0

        if i == 1
            % Velocity computation
            w(:,i) = Robot.k(:,i)*q_dot(i);
            v(:,i) = [0 0 0]';
            % Acceleration computation
            w_dot(:,i) = Robot.k(:,i)*q_2dot(i);
            v_dot(:,i) = [0 0 0]';
        else 
            ri = Robot.Q(:,i-1) - Robot.P(:,i-1);                          % Position of Pi with respet to Pi-1
            % Velocity computation
            w(:,i) = w(:,i-1) + Robot.k(:,i)*q_dot(i);
            v(:,i) = v(:,i-1) + cross(w(:,i-1),ri);
            % Acceleration computation
            w_dot(:,i) = w_dot(:,i-1) + cross(w(:,i-1), Robot.k(:,i))*q_dot(i) + Robot.k(:,i)*q_2dot(i);
            v_dot(:,i) = v_dot(:,i-1) + cross(w_dot(:,i-1),ri) + cross(w(:,i-1),cross(w(:,i-1),ri));
        end

    % Prismatic joints
    elseif Robot.Joint(i) == 1

        if i == 1
            % Velocity computation
            w(:,i) = [0 0 0]';
            v(:,i) =  Robot.k(:,i)*q_dot(i);
            % Accelerations computation
            w_dot(:,i) = [0 0 0]';
            v_dot(:,i) = Robot.k(:,i)*q_2dot(i);
        else 
            ri = (Robot.Q(:,i-1) - Robot.P(:,i-1)) + Robot.k(:,i)*q(i);    % Position of Pi with respet to Pi-1
            % Velocity computation
            w(:,i) = w(:,i-1);
            v(:,i) = v(:,i-1) + cross(w(:,i-1), ri) + Robot.k(:,i)*q_dot(i);
            % Acceleration computation
            w_dot(:,i) = w_dot(:,i-1);
            v_dot(:,i) = v_dot(:,i-1) + cross(w_dot(:,i-1),ri) + cross(w(:,i-1),cross(w(:,i-1),ri)) + cross(2*w(:,i-1),Robot.k(:,i))*q_dot(i) + Robot.k(:,i)*q_2dot(i);
        end

    end
    v_C(:,i) = v(:,i) + cross(w(:,i),Robot.C(:,i) - Robot.P(:,i));
    v_Cdot(:,i) = v_dot(:,i) + cross(w_dot(:,i), Robot.C(:,i) - Robot.P(:,i)) + cross(w(:,i),cross(w(:,i),Robot.C(:,i) - Robot.P(:,i)));
end

% Backward
Qplus = [Robot.P(:,2:Robot.nJoints) Robot.Q(:,Robot.nJoints)];

for i = Robot.nJoints:-1:1

    % External forces and moments
    Fext = [0 0 0]';
    Mext = [0 0 0]';
    
    % Forces and moments calculated with respect to link i+1
    Fprec = F;
    Mprec = M;
    
    % Dynamic force
    D(:,i) = Robot.m(i)*v_Cdot(:,i);
    % Dynamic tourque
    Delta(:,i) = Robot.Ic(:,:,i)*w_dot(:,i) + cross(w(:,i),Robot.Ic(:,:,i)*w(:,i));
    % Force and moment computation
    F = - Robot.m(i)*g + Fprec + D(:,i) + Fext;
    M =  - cross((Robot.P(:,i) - Robot.C(:,i)),F) + cross((Qplus(:,i) - Robot.C(:,i)), Fprec) + Mprec + Delta(:,i) + Mext;
    
    if Robot.Joint(i) == 0
        Tau(i) = M'*Robot.k(:,i);
    elseif Robot.Joint(i) == 1
        Tau(i) = F'*Robot.k(:,i);
    end
end

end