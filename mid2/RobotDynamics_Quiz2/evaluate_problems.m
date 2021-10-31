% Initialize the workspace.
init_workspace

fprintf('Evaluating user implementations vs. solutions... \n')

%% Setup
% generalized coordinates
gc = generate_gc;

% Initialize the parameters for the mid-term exam.
params = init_params;

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

%% Q 1.
disp('Running Q 1...');

% Equation of the Motion
eom = Q1_generate_eom(gc, kin, params, jac);

% Test settings
N = 100;   % number of tests
tol = 1e-5; % test tolerance

% --- Convenience functions ---
almostequal = @(x1, x2) all(abs(x1 - x2) < tol);
disp_correct = @(fun) fprintf('%s: correct \n', fun);
disp_incorrect = @(fun) fprintf('%s: incorrect \n', fun);
% -----------------------------

% Evaluate M_fun.m
q_eval = randn(3,1);
user_output = feval('M_fun', q_eval);

if check_mass_matrix(user_output) == 1
    disp_correct('(+1) M_fun dimension and symmetricity');
else
    disp_incorrect('(+0) M_fun dimension and symmetricity');
end

correct = 0;
for i = 1:N
    q_eval = randn(3,1);
    user_output = feval('M_fun', q_eval);
    solution = feval('M_fun_solution', q_eval);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end

if correct == N
    disp_correct('(+1) M_fun values');
else
    disp_incorrect('(+0) M_fun values');
end

% Evaluate g_fun.m
correct = 0;
for i = 1:N
    q_eval = randn(3,1);
    user_output = feval('g_fun', q_eval);
    solution = feval('g_fun_solution', q_eval);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
if correct == N
    disp_correct('(+1) g values');
else
    disp_incorrect('(+0) g values');
end

% Evaluate b_fun.m
correct = 0;
for i = 1:N
    q_eval = randn(3,1);
    qd_eval = randn(3,1);
    user_output = feval('b_fun', q_eval, qd_eval);
    solution = feval('b_fun_solution', q_eval, qd_eval);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
if correct == N
    disp_correct('(+1) b values');
else
    disp_incorrect('(+0) b values');
end

%% Q 2.
disp('Running Q 2...');

correct = 0;
N = 10;
for i = 1:N
    gc_eval.q = randn(3,1);
    gc_eval.dq = randn(3,1);
    tau_eval = randn(3,1);
    [~, user_output] = Q2_forward_simulator(gc_eval, tau_eval, 0.01);
    [~, solution] = Q2_forward_simulator_solution(gc_eval, tau_eval, 0.01);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end

if correct == N
    disp_correct('(+2) forward_simulator');
else
    disp_incorrect('(+0) forward_simulator');
end

%% Q 3.
disp('Running Q 3...');

correct = 0;
N = 10;
for i = 1:N
    gc_eval.q = randn(3,1);
    gc_eval.dq = randn(3,1);
    q_des = randn(3,1);
    dq_des = randn(3,1);
    user_output = Q3_gravity_compensation(params, gc_eval, q_des, dq_des);
    solution = Q3_gravity_compensation_solution(params, gc_eval, q_des, dq_des);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
if correct == N
    disp_correct('(+2) gravity_compensation');
else
    disp_incorrect('(+0) gravity_compensation');
end

%% Q4
close all
disp('Running Q 4...');
gc.q = [-pi/1.2;pi/2.0;pi/2.0];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];
disp('If end effector is within the green circle, +2');

figure
for sim_step = 1:params.Q4_targetN
    % control input
    tau = Q4_task_space_inverse_dynamics(params, gc, ...
        params.Q4_I_r_IF(1:3, sim_step), ...
        params.Q4_I_dr_IF(1:3, sim_step), ...
        params.Q4_eul_IF_des, ...
        zeros(3,1));
    
    % vis
    plot(params.Q4_I_r_IF(2,:),params.Q4_I_r_IF(3,:)); hold on;
    plot(params.Q4_I_r_IF(2, sim_step),  params.Q4_I_r_IF(3, sim_step), 'go', 'MarkerSize',10); hold on;
    gc = visualize_solution(gc, params, tau, 0, zeros(6,1));
    pause(params.control_dt);
end

%% Q5
close all
disp('Running Q 5...');

gc_eval.q = zeros(3,1);
gc_eval.dq = zeros(3,1);
des = randn(3,1);
des2 = randn(3,1);
des_Fz = -40.0 + 3.0 * rand;
[tau, SM, SF, I_FF] = Q5_hybrid_operational_space_control(params, gc_eval, des, des2, des_Fz);

if check_selection_matrices(SM, SF)
    disp_correct('(+1) selection matrix');
else
    disp_incorrect('(+0) selection matrix');
end

if check_force(I_FF, params, des_Fz)
    disp_correct('(+1) feed-forward force');
else
    disp_incorrect('(+0) feed-forward force');
end

disp('If the foot slips in [0, -1, 0] direction while keeping 45 degrees to the surface, +1');
gc.q = [-2.4052; 1.5568; 0.0673];
gc.dq = [0.0; 0.0; 0.0];
figure
for sim_step = 1:params.Q5_targetN
    % control input
    [tau, SM, SF, I_FF] = Q5_hybrid_operational_space_control(params, gc, params.Q5_I_r_IF(1:3, sim_step), params.Q4_eul_IF_des, -40.0);
    % vis
    gc= visualize_solution(gc, params, tau, 1, I_FF);
    foot_position =  I_r_IF_fun(gc.q);
    if foot_position(2) < 0.1 && foot_position(3) < 0.05
    break;
    end
    pause(params.control_dt);
end
