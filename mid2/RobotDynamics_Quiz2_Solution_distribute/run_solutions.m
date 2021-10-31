init_workspace

%% Setup
% generalized coordinates
gc = generate_gc;

% Initialize the parameters for the mid-term exam.
params = init_params;

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);



figure
%% Gravity Compensation
disp('Gravity Compensation...');
gc.q = [-pi/1.2;pi/2.5;pi/4];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];
q_des = [-pi/1.2;pi/2.0;pi/2.0];
dq_des = [0.0; 0.0; 0.0];
target = I_r_IF_fun(q_des);

for sim_step = 1:50
   % control input
   tau = Q3_gravity_compensation_solution(params, gc, q_des, dq_des);
     
   % vis
   plot(target(2), target(3), 'go', 'MarkerSize',10); hold on;
   gc =  visualize_solution(gc, params, tau, 0, zeros(3,1));
   pause(params.control_dt);
end
%% Operational Space Inverse Dynamics
disp('Task Space Inverse Dynamics...');
gc.q = [-pi/1.2;pi/2.0;pi/2.0];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

for sim_step = 1:params.Q4_targetN
    % control input
    tau = Q4_task_space_inverse_dynamics_solution(params, gc, ...
        params.Q4_I_r_IF(1:3, sim_step), ...
        params.Q4_I_dr_IF(1:3, sim_step), ...
        params.Q4_eul_IF_des, ...
        zeros(3,1));
    
    % vis
    plot(params.Q4_I_r_IF(2,:),params.Q4_I_r_IF(3,:)); hold on;
    plot(params.Q4_I_r_IF(2, sim_step),  params.Q4_I_r_IF(3, sim_step), 'go', 'MarkerSize',10); hold on;
    gc = visualize_solution(gc, params, tau, 0, zeros(3,1));
    pause(params.control_dt);
end

%% Q5
disp('Hybrid Operational Space Control...');
gc.q = [-2.4052; 1.5568; 0.0673];
gc.dq = [0.0; 0.0; 0.0];
for sim_step = 1:params.Q5_targetN
    % control input
    [tau, I_FF] = Q5_hybrid_operational_space_control_solution(params, gc, params.Q5_I_r_IF(1:3, sim_step), params.Q4_eul_IF_des, -40.0);
   
    % vis
    gc= visualize_solution(gc, params, tau, 1, I_FF);
    foot_position =  I_r_IF_fun(gc.q);
    if foot_position(2) < 0.1 && foot_position(3) < 0.05
    break;
    end
    pause(params.control_dt);
end
