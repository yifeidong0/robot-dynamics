% generate jacobians
function jac = generate_jac(gc, kin, params)
% By calling:
%   jac = generate_jac(gen_cor, kin, dyn)
% a struct 'jac' is returned that contains the translation and rotation
% jacobians of the center of masses

%% Setup
q = gc.q;
dq = gc.dq;

T_Ik = kin.T_Ik;
R_Ik = kin.R_Ik;

T_IF = kin.T_IF;
R_IF = kin.R_IF;

I_r_IF = kin.I_r_IF;

k_r_ks = params.k_r_ks;

jac.I_Jp_s = cell(3,1);
jac.I_Jr = cell(3,1);

%% Compute link jacobians

I_Jp_s = cell(3,1); % gc dim
I_Jr = cell(3,1);

for k= 1:3
    % create containers
    I_Jp_s{k} = sym(zeros(3,3));
    I_Jr{k} = sym(zeros(3,3));
    
    % translational jacobian at the center of gravity s in frame I
    I_r_ks = [eye(3) zeros(3,1)]*T_Ik{k}*[k_r_ks{k};1];
    I_Jp_s{k} = jacobian(I_r_ks, q);
    
    % rotational jacobian in frame I
    if k == 1
        I_Jr{k}(1:3,1) = R_Ik{1} * [0; 0; 1];
    else
        % copy columns of k-1 jacobian
        I_Jr{k} = I_Jr{k-1};
        
        % evaluate new column
        I_Jr{k}(1:3,k) = R_Ik{k} * [0; 0; 1];
    end
    
    % simplify expressions
    I_Jp_s{k} = simplify(I_Jp_s{k});
    I_Jr{k} = simplify(I_Jr{k});
end

% Compute the end effector jacobians in frame I
I_Jp_F = simplify(jacobian(I_r_IF, q));
I_Jr_F = I_Jr{3};

% Compute the time derivative of the end effector Jacobians
I_dJp_F = simplify(dAdt(I_Jp_F,q,dq));
I_dJr_F = simplify(dAdt(I_Jr_F,q,dq));


% % Generate function files from symbolic expressions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating dJe file... ');
matlabFunction(I_Jp_F, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jp_F_fun'));
matlabFunction(I_dJp_F, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJp_F_fun'));

matlabFunction(I_Jr_F, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jr_F_fun'));
matlabFunction(I_dJr_F, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJr_F_fun'));

fprintf('done!\n')


% % Store jacobians in output struct
jac.I_Jp_s = I_Jp_s;
jac.I_Jr = I_Jr;
jac.I_Jp_F = I_Jp_F;
jac.I_Jr_F = I_Jr_F;
jac.I_dJp_F = I_dJp_F;
jac.I_dJr_F = I_dJr_F;

end