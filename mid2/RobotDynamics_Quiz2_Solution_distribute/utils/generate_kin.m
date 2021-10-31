% generate kinematics
function kin = generate_kin(q, params)
%% Create kinematics container
kin = struct();

% homogeneous transformations from frame k to frame k-1 = j
% kin.T_jk = cell(3,1);

% homogeneous transformations from frame k to the inertial frame
kin.T_Ik = cell(3,1);

% rotation matrices from frame k to the inertial frame
kin.R_Ik = cell(3,1);

%% Homogeneous transformations

% homogeneous transformations from frame k to frame I

kin.T_Ik{1} = T_I0_fun(q, params) * T_01_fun(q, params);
kin.T_Ik{2} = simplify(kin.T_Ik{1}*T_12_fun(q, params));
kin.T_Ik{3} = simplify(kin.T_Ik{2}*T_23_fun(q, params));

% rotation matrices from frame k to frame I
kin.R_Ik{1} = kin.T_Ik{1}(1:3,1:3);
kin.R_Ik{2} = kin.T_Ik{2}(1:3,1:3);
kin.R_Ik{3} = kin.T_Ik{3}(1:3,1:3);

%% Endeffector
% end-effector homogeneous transformation and position
kin.T_IF = simplify(kin.T_Ik{3} * T_3F_fun(q,params));
kin.R_IF =  kin.T_IF(1:3,1:3);
kin.I_r_IF = kin.T_IF(1:3,4);

%% Matalb functions (TODO)
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
% 
fprintf('Generating end-effector position file... ');
matlabFunction(kin.I_r_IF, 'vars', {q}, 'file', strcat(dpath,'/I_r_IF_fun'));
fprintf('done!\n')

fprintf('Generating forward kinematics file... ');
matlabFunction(kin.T_IF, 'vars', {q}, 'file', strcat(dpath,'/T_IF_fun'));
fprintf('done!\n')

end
