function [tau, I_F_F ] = Q5_hybrid_operational_space_control_solution( params, gc, I_r_Fd, I_eul_Fd, I_F_Fz)
% Operational-space inverse dynamics controller 
% with a PD stabilizing feedback term and a desired end-effector force.
%
% I_r_Fd --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% I_eul_Fd --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% I_F_Fz --> a scalar value which describes a desired force in the z
% direction of the contact frame
%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_F; % P gain matrix for the end-effector (6x6 diagonal matrix)
kd = params.kd_F; % D gain matrix for the end-effector (6x6 diagonal matrix)

% Friction Coefficient
mu_s = params.mu_s; % Static friction coefficient
mu_k = params.mu_k; % Kinetic friction coefficient

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

T_IP = T_IP_fun(params); % Homogeneous transformation from frame P to frame I

% Project the joint-space dynamics to operational space
j_invm = I_J_F/M;
lambda = pseudoInverseMat(j_invm*I_J_F', 0.001); % Inertia matrix in the operational space
mu = lambda*(j_invm*b - I_dJ_F*dq); % Nonlinear term in the operational space
p =  lambda*j_invm*g;  % Gravity term in the operational space

% Define error orientation using the rotational vector parameterization.
C_IF_des = eulAngXyzToRotMat(I_eul_Fd);
C_err = C_IF_des*C_IF';
orientation_error = rotMatToRotVec(C_err);

% Define the pose error.
pos_err = [I_r_Fd - I_r_IF;
           orientation_error];
       
% Target velocity of the end effector are zero.
I_dr_Fd = zeros(3,1);
I_w_Fd = zeros(3,1);
       
%% Desired end-effector force !!NOTE: Do not rename the variable I_F_F. It is used in grading!!
I_F_F = zeros(6,1); % Desired end-effector force in inertial frame
I_F_F(3) = I_F_Fz; % Normal force
I_F_F(2) =  - (params.mu_s * abs(I_F_Fz) + 0.5); % Slightly larger than the static friction in -y direction.

%% Selection Matrix !!NOTE: Do not rename the variable SM, SF. They are used in grading!!
SM = zeros(6,6);
SF = zeros(6,6);

sigma_p = diag([1.0, 0.0, 0.0]); % No motion control in yz plane
sigma_r = diag([1.0, 1.0, 1.0]); % Orientation should be controlled
C = T_IP(1:3, 1:3);

SM = [C * sigma_p * C', zeros(3); zeros(3), C * sigma_r * C'];
SF = [C * (eye(3) - sigma_p) * C', zeros(3);...
    zeros(3),C * (eye(3) - sigma_r) * C'];

%% Hybrid Operational Space Controller !!NOTE: Do not rename the variable tau. It is used in grading!!
tau = zeros(3,1); % TODO
vel_err = [I_dr_Fd - I_Jp_F * dq; I_w_Fd- I_Jr_F * dq];  
w_des = (kp*pos_err + kd*vel_err);
I_F_F_des = lambda * SM * w_des ...
          + SF*I_F_F + mu + p;

% Map the desired force back to the joint-space torques
tau = I_J_F'*I_F_F_des;

end
