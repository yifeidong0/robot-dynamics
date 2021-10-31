% Add folders to path
thisfile = which(mfilename);
exercisefolder = fileparts(thisfile);
cd(exercisefolder);
addpath(genpath(exercisefolder));
clear thisfile exercisefolder;

%% Link properties

params = struct;

% link lengths
params.l0 = 0.3;
params.l1 = 0.5;
params.l2 = 0.25;
params.l3 = 0.25;

params.m = cell(3,1); % Mass of link k
params.k_r_ks = cell(3,1); % position of COM of link k in link k frame
params.k_I_s = cell(3,1); % Inertia tensor of link k with respect to frame k
params.k_n_k = cell(3,1); % rotation axis unit vector

% Link 1
params.k_r_ks{1} = [0.1; 0; 0];
params.m{1} = 1.0;
params.k_I_s{1} = [0.1 0 0;
                   0 0.2 0;
                   0 0 0.3];
params.k_n_k{1} = [0; 0; 1];
               
% Link 2
params.k_r_ks{2} = [0.2; 0; 0];
params.m{2} = 1.5;
params.k_I_s{2} = [0.2 0 0;
                   0 0.2 0;
                   0 0 0.3];
params.k_n_k{2} = [0; 0; 1];
               
% Link 3
params.k_r_ks{3} = [0.05; 0; 0];
params.m{3} = 0.5;
params.k_I_s{3} = [0.4 0 0;
                   0 0.4 0;
                   0 0 0.1];
params.k_n_k{3} = [0; 0; 1];
               
%% Jacobians
I_Jr = cell(3,1); % rotation jacobian in I frame

for k=1:3
    I_Jr{k} = zeros(3, 3);
    for j=1:k
        I_Jr{k}(3,j) = 1.0;
    end
end

I_Jp_s = cell(3,1); % position jacobian of link COM in I frame

q_sym = sym('q', [3 1]);
dq_sym = sym('dq', [3 1]);

I_r_1s = [eye(3) zeros(3,1)]*T_I0_solution(q_sym, params)*T_01_solution(q_sym, params)*[params.k_r_ks{1};1];
I_Jp_1s = matlabFunction(simplify(jacobian(I_r_1s, q_sym)));
I_Jp_s{1} = @(q) I_Jp_1s(q(1));

I_r_2s = [eye(3) zeros(3,1)]*T_I0_solution(q_sym, params)*T_01_solution(q_sym, params)*T_12_solution(q_sym, params)*[params.k_r_ks{2};1];
I_Jp_2s = matlabFunction(simplify(jacobian(I_r_2s, q_sym)));
I_Jp_s{2} = @(q) I_Jp_2s(q(1), q(2));

I_r_3s = [eye(3) zeros(3,1)]*T_I0_solution(q_sym, params)*T_01_solution(q_sym, params)*T_12_solution(q_sym, params)*T_23_solution(q_sym, params)*[params.k_r_ks{3};1];
I_Jp_3s = matlabFunction(simplify(jacobian(I_r_3s, q_sym)));
I_Jp_s{3} = @(q) I_Jp_3s(q(1), q(2), q(3));

I_r_E = [eye(3) zeros(3,1)]*T_I0_solution(q_sym, params)*T_01_solution(q_sym, params)*T_12_solution(q_sym, params)*T_23_solution(q_sym, params)*[params.l3; 0; 0;1];
I_Jp_E_mfun = matlabFunction(simplify(jacobian(I_r_E, q_sym)));
I_Jp_E = @(q) I_Jp_E_mfun(q(1),q(2),q(3));

%% Jacobian derivatives
I_dJr = cell(3,1);
for k=1:3
    I_dJr{k} = zeros(3,3);
end

I_dJp_s = cell(3,1);

I_dJp_1s = matlabFunction(simplify(dAdt(simplify(jacobian(I_r_1s, q_sym)), q_sym, dq_sym)));
I_dJp_s{1} = @(q,dq) I_dJp_1s(dq(1),q(1));

I_dJp_2s = matlabFunction(simplify(dAdt(simplify(jacobian(I_r_2s, q_sym)), q_sym, dq_sym)));
I_dJp_s{2} = @(q,dq) I_dJp_2s(dq(1),dq(2),q(1),q(2));

I_dJp_3s = matlabFunction(simplify(dAdt(simplify(jacobian(I_r_3s, q_sym)), q_sym, dq_sym)));
I_dJp_s{3} = @(q,dq) I_dJp_3s(dq(1),dq(2),dq(3),q(1),q(2),q(3));

I_dJp_E_mfun = matlabFunction(simplify(dAdt(simplify(jacobian(I_r_E, q_sym)), q_sym, dq_sym)));
I_dJp_E = @(q,dq) I_dJp_E_mfun(dq(1),dq(2),dq(3),q(1),q(2),q(3));