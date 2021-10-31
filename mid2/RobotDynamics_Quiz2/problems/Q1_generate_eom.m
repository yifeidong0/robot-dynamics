% generate equations of motion
function eom = Q1_generate_eom(gc, kin, params, jac)
% By calling:
%   eom = generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion. These are additionally
% converted to matlab scripts.

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (3x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (3x1 cell)->(3x3 sym)

k_I_s = params.k_I_s;      % Inertia tensor of body k in frame k (3x1 cell)->(3x3 sym)
m = params.m;              % Mass of body k (3x1 cell)->(1x1 double)
I_g_acc = params.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = params.k_r_ks;    % CoM location of body k in frame k (3x1 cell)->(3x1 double)

I_Jp_s = jac.I_Jp_s;    % CoM Positional Jacobian in frame I (3x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr;        % CoM Rotational Jacobian in frame I (3x1 cell)->(3x6 sym)
I_dJp_s = cell(3,1);
I_dJr = cell(3,1);

eom.M = sym(zeros(3,3));
eom.g = sym(zeros(3,1));
eom.b = sym(zeros(3,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = zeros(3,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:3
    M = M + m{k}*I_Jp_s{k}'*I_Jp_s{k} + I_Jr{k}'*R_Ik{k}*k_I_s{k}*R_Ik{k}'*I_Jr{k};
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = simplify(M);
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(3,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:3
    g = g - I_Jp_s{k}' * m{k} * I_g_acc; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = simplify(g); % Allow more time for more simplified solution
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(3,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:3
    I_sk = R_Ik{k}*k_I_s{k}*R_Ik{k}';
    omega = I_Jr{k} * dq;
    I_dJp_s{k} = simplify(dAdt(I_Jp_s{k},q, dq));
    I_dJr{k} = simplify(dAdt(I_Jr{k},q, dq));

    b = b + I_Jp_s{k}' * m{k} * I_dJp_s{k} * dq ...
          + I_Jr{k}' * I_sk * I_dJr{k} * dq ...
          + I_Jr{k}' * cross(omega, I_sk * omega);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = simplify(b);
fprintf('done!\n');

%% Generate matlab functions !!NOTE: Do not modify this part. It is used in grading!!
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating eom scripts... ');
fprintf('M... ');
matlabFunction(M, 'vars', {q}, 'file', strcat(dpath,'/M_fun'), 'Optimize', true);
fprintf('g... ');
matlabFunction(g, 'vars', {q}, 'file', strcat(dpath,'/g_fun'), 'Optimize', true);
fprintf('b... ');
matlabFunction(b, 'vars', {q, dq}, 'file', strcat(dpath,'/b_fun'), 'Optimize', true);
fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end
