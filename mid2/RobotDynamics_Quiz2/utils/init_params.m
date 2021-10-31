function params = init_params()
% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.2;
params.l1 = 0.2;
params.l2 = 0.3;
params.l3 = 0.04;
params.l4 = 0.28;
params.l5 = 0.15;
params.L  = 0.5;

%% Gains
params.kp_j = diag([40 10 5]);
params.kd_j =  diag([5.0 5.0 0.5]);

params.kp_F = diag([60 60 60 60 60 60]);
params.kd_F = diag([10 10 10 10 10 10]);

%% Link properties
params.m = cell(3,1);
params.k_r_ks = cell(3,1);
params.k_I_s = cell(3,1);

params.m{1} = 4.5;
params.k_r_ks{1} = [0.5 * params.l2; 0.0; 0.0];
params.k_I_s{1} = [0.060 0 0;
    0 0.05 0;
    0 0 0.045];

params.m{2} = 3.5;
params.k_r_ks{2} = 1.0/(params.l3 + params.l4) * [0.5 * params.l4 * params.l4;...
    0.5 * params.l3 * params.l3 + params.l3 * params.l4; 0.0];
params.k_I_s{2} = [0.02 0 0;
    0 0.02 0;
    0 0 0.03];


params.m{3} = 2.5;
params.k_r_ks{3} = [0.5 * params.l5; 0.0; 0.0];
params.k_I_s{3} = [0.01 0 0;
    0 0.01 0;
    0 0 0.02];

%% Gravity
params.I_g_acc = [0; 0; -9.81];

%% Simulation params
% Sampling time used for discrete integration steps, and for visualization
params.simulation_dt = 0.01;
params.control_dt = 0.05;

%% Terrain property
params.mu_s = 0.5;
params.mu_k = 0.4;

%% Targets
params.Q4_targetN = 70;
params.Q4_eul_IF_des = [pi/4;0;0];
params.Q4_I_r_IF = zeros(3, params.Q4_targetN);
params.Q4_I_dr_IF = zeros(3, params.Q4_targetN);

step_size = 0.5 * pi / params.Q4_targetN;
offset = 0;
radi = 0.15;
T_IP = T_IP_fun(params);
center = T_IP(2:3,4) - [radi;0];


for sim_step = 1:params.Q4_targetN
    params.Q4_I_r_IF(1:3, sim_step) = [params.l0; ...
        center(1) + radi * sin(step_size * sim_step - offset); ...
        center(2) + radi * cos(step_size * sim_step - offset)];
    
    params.Q4_I_dr_IF(1:3, sim_step) = [0; ...
        radi * step_size *  cos(step_size * sim_step - offset); ...
        - radi * step_size *  sin(step_size * sim_step - offset)];
end

params.Q5_targetN = 100;
params.Q5_I_r_IF = [params.l0; T_IP(2,4); T_IP(3,4)];
params.Q5_eul_IF_des = [pi/4;0;0];

for sim_step = 1:params.Q4_targetN
    params.Q5_I_r_IF(1:3, sim_step) = [params.l0; ...
        T_IP(2,4) - 0.1 * sim_step; ...
        T_IP(3,4)];
    
%     params.Q4_I_dr_IF(1:3, sim_step) = [0; ...
%         radi * step_size *  cos(step_size * sim_step - offset); ...
%         - radi * step_size *  sin(step_size * sim_step - offset)];
end


end