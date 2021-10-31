% Use this file to code your solution.

% The following modelling quantities are already provided (shown exemplary
% for link 1 and random joint angle)

q = [pi/4, pi/8, pi/2]'; % joint configuration for question 1
dq = [0.1, 0.2, -0.2]'; % joint speed for question 1

params.l1; % length of link 1
params.m{1}; % mass of link 1
params.k_r_ks{1}; % position of center of mass of link 1 in link 1 frame
params.k_I_s{1}; % intertia tensor of link 1 in link 1 frame
params.k_n_k{1}; % rotation axis of joint 1 in link 1 frame
I_Jr{1}; % rotational Jacobian of link 1 in I frame
I_dJr{1}; % time derivative of I_Jr{1}
I_Jp_s{1}(q); % positional Jacobian of center of mass of link 1 in I frame
I_dJp_s{1}(q,dq); % time derivative of I_Jp_s{1}
I_Jp_E(q); % positional Jacobian of the end-effector in I frame
I_dJp_E(q,dq); % time derivative of I_Jp_E


%% For question 2:
q = [pi/4, -pi/4, pi/4]';
dq = [0, 0, 0]';

M = [1.6369    0.6641    0.1182;
     0.6641    0.5013    0.1057;
     0.1182    0.1057    0.1013];
 
b = [0 0 0]';

g = [11.9731, 4.3427, 0.1734]';

J = I_Jp_E(q);
dJ = I_dJp_E(q,dq);
