function [ tau ] = Q3_gravity_compensation_solution(params, gc, q_des, dq_des)
% Joint space PD controller with gravity compensation
%
% gc -- > generalized coordinates
% q_des --> desired q
% dq_des --> desired dq

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_j; % P gain matrix for joints (3x3 diagonal matrix)
kd = params.kd_j; % D gain matrix for joints (3x3 diagonal matrix)

% The control action has a gravity compensation term, as well as a PD
% feedback action which depends on the current state and the desired
% configuration.

%% Gravity compensation  !!NOTE: Do not rename the variable tau. It is used in grading!!

tau = kp * (q_des - q) ...
      + kd * (dq_des - dq)...
      + g;
end