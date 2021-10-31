function [ tau ] = control_op_space_hybrid( I_r_IE_des, eul_IE_des, q, dq, I_F_E_x )
% CONTROL_OP_SPACE_HYBRID Operational-space inverse dynamics controller 
% with a PD stabilizing feedback term and a desired end-effector force.
%
% I_r_IE_des --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% eul_IE_des --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% q --> a vector in R^n of measured joint positions
% q_dot --> a vector in R^n of measured joint velocities
% I_F_E_x --> a scalar value which describes a desired force in the x
%   direction

% Design the control gains
kp = 1; % TODO
kd = 1; % TODO

% Desired end-effector force
I_F_E = [I_F_E_x, 0.0, 0.0, 0.0, 0.0, 0.0]';

% Find jacobians, positions and orientation
I_J_e = I_Je_fun_solution(q);
I_dJ_e = I_dJe_fun_solution(q, dq);
T_IE = T_IE_fun_solution(q);
I_r_IE = T_IE(1:3, 4);
C_IE = T_IE(1:3, 1:3);

% Define error orientation using the rotational vector parameterization.
C_IE_des = eulAngXyzToRotMat(eul_IE_des);
C_err = C_IE_des*C_IE';
orientation_error = rotMatToRotVec_solution(C_err);

% Define the pose error.
chi_err = [I_r_IE_des - I_r_IE;
           orientation_error];

% Project the joint-space dynamics to the operational space
% TODO
lamda = inv(I_J_e/eom.m*I_J_e');
mue = lamda*I_J_e/eom.m*eom.b - lamda*I_dJ_e*q_dot;
p = lamda*I_J_e/eom.m*eom.g;

% Define the motion and force selection matrices.
% TODO
sigmaMp = [0 0 0;0 1 0;0 0 1];
sigmaMr = [1 0 0;0 0 0;0 0 0];
sigmaFp = [1 0 0;0 0 0;0 0 0];
sigmaFr = [0 0 0;0 1 0;0 0 1];

mat_zero = zeros(3,3);
Sm = [C_IE'*sigmaMp*C_IE,mat_zero; mat_zero,C_IE'*sigmaMr*C_IE] ;
Sf = [C_IE'*(eye(3)-sigmaFp)*C_IE,mat_zero; mat_zero,C_IE'*(eye(3)-sigmaFr)*C_IE] ;

% Design a controller which implements the operational-space inverse
% dynamics and exerts a desired force.
w_dot = kp*chi_err+kd*ones(6,1);%desired w_star and w ???

tau = I_J_e'*(lamda*Sm*w_dot+Sf*I_F_E+mue+p); % TODO

end
