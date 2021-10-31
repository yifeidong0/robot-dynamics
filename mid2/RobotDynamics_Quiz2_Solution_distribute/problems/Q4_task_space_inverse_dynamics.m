function [ tau ] = Q4_task_space_inverse_dynamics(params, gc, I_r_Fd, I_dr_Fd, I_eul_Fd, I_w_Fd)
% task space inverse dynamics controller with a PD 
% stabilizing feedback term.
%
% I_r_Fd --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% I_eul_Fd --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% I_dr_Fd --> a vector in R^3 which describes the desired velocity of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% I_w_Fd -->a vector in R^3 which describes the desired angular velocity of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_F; % P gain matrix for the end-effector (6x6 diagonal matrix)
kd = params.kd_F; % D gain matrix for the end-effector (6x6 diagonal matrix)

% Find jacobians, positions and orientation based on the current
% measurements.
I_Jp_F = I_Jp_F_fun(q); % Position Jacobian of the end-effector (3x3)
I_dJp_F = I_dJp_F_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_Jr_F = I_Jr_F_fun(q); % Rotational Jacobian of the end-effector (3x3)
I_dJr_F = I_dJr_F_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)
I_J_F = [I_Jp_F; I_Jr_F]; % Jacobian of the end-effector
I_dJ_F = [I_dJp_F; I_dJr_F]; % Time derivative of the Jacobian of the end-effector

T_IF = T_IF_fun(q); % Homogeneous transformation from frame F to frame I
I_r_IF = T_IF(1:3, 4);
C_IF = T_IF(1:3, 1:3);

% Define error orientation using the rotational vector parameterization.
C_IF_des = eulAngXyzToRotMat(I_eul_Fd);
C_err = C_IF_des*C_IF';
orientation_error = rotMatToRotVec(C_err);

% Define the pose error.
pos_err = [I_r_Fd - I_r_IF;
           orientation_error];


%% Task-space Inverse Dynamics Controller !!NOTE: Do not rename the variable tau. It is used in grading!!
% Define the velocity error of the end effector.       
vel_err = zeros(6,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: implement your solution here %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau = zeros(3, 1); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: implement your solution here %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end