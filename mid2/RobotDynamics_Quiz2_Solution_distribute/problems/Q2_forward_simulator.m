function [gc_next, ddq] = Q2_forward_simulator(gc, tau, time_step)
% Compute generalized acceleration of the system
%
% gc -- > generalized coordinates
% tau --> generalized force acting on the system
% time_step --> simulation time step

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

%% Compute generalized acceleration of the system
ddq = zeros(3,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: implement your solution here %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Forward Integration !!NOTE: Do not modify this part.!!
gc_next.dq = (dq + ddq * time_step);
gc_next.q = (q + (dq + gc_next.dq) * 0.5 * time_step);

end
