%% preparation
clear all;
init_workspace

%% question 1
q = [pi/4, pi/8, pi/2]'; % example joint configuration
dq = [0.1, 0.2, -0.2]'; % example joint configuration

% mass matrix
M = zeros(3,3);

T_I1 = T_I0_solution(q_sym, params) * T_01_solution(q_sym, params);
T_I2 = T_I1 * T_12_solution(q_sym, params);
T_I3 = T_I2 * T_23_solution(q_sym, params);

R_Ik = cell(3,1);
R_Ik{1} = T_I1(1:3, 1:3);
R_Ik{2} = T_I2(1:3, 1:3);
R_Ik{3} = T_I3(1:3, 1:3);

for k = 1:3
    M = M + params.m{k}*I_Jp_s{k}(q)'*I_Jp_s{k}(q)...
          + I_Jr{k}'*R_Ik{k}*params.k_I_s{k}*R_Ik{k}'*I_Jr{k};
end
M = double(M);
disp("M = ")
disp(M)

% nonlinear terms
b = zeros(3,1);
for k = 1:3
    I_sk = R_Ik{k}*params.k_I_s{k}*R_Ik{k}';
    omega = I_Jr{k} * dq;
    
    b = b + I_Jp_s{k}(q)' * params.m{k} * I_dJp_s{k}(q,dq) * dq ...
          + I_Jr{k}' * I_sk * I_dJr{k} * dq ...
          + I_Jr{k}' * cross(omega, I_sk * omega);
end
b = double(b);
disp("b = ")
disp(b)

% gravity
g = zeros(3,1);
for k=1:3
    g = g - I_Jp_s{k}(q)' * params.m{k} * [0, -9.81, 0]';  %[0, -9.81]????
end
g = double(g);
disp("g = ")
disp(g)

%% Question 2
q = [pi/4, -pi/4, pi/4]';
dq = [0, 0, 0]';

M = [1.6369    0.6641    0.1182;
     0.6641    0.5013    0.1057;
     0.1182    0.1057    0.1013];
 
b = [0 0 0]';

g = [11.9731, 4.3427, 0.1734]';

%% question 2.1
tau = g - I_Jp_E(q)' * 5.0 * [0, -9.81, 0]';
disp("Ex 2.1 tau =")
disp(tau)

%% question 2.2

J = I_Jp_E(q);
J = J(2,:); % only vertical direction of interest
dJ = I_dJp_E(q,dq);
dJ = dJ(2,:); % only vertical direction of interest

% Option A --> only pseudoinverse
ddq1 = pinv(J) * (1 - dJ*dq);

% Option B --> null space and pseudoinverse
A1 = [M,-eye(3)];
b1 = [-g-b];
A2 = [J,[0,0,0]];
b2 = [1-dJ*dq];
A3 = [eye(3),zeros(3,3)];
b3 = zeros(3,1);
%first coordinate blocked/rupture case:
% A4 = [1,zeros(1,5)]; %blocked
% A4 = [zeros(1,3),[1,0,0]]; %rupture
% b4 = 0;
N = null([A1;A2]); %The null space of a matrix contains vectors x that satisfy Ax=0.
x = pinv([A1;A2])*[b1;b2]+N*pinv(A3*N)*(b3-A3*pinv([A1;A2])*[b1;b2]);
%x = pinv([A1;A2;A3])*[b1;b2;b3]; with equal priority
tau2 = x(4:6);
ddq2 = x(1:3);

% Option C --> quadprog
% ddot(x_e) = dJ *dq + J * ddq -- required to be = 1
%Solver for quadratic objective functions with linear constraints.
%quadprog finds a minimum for a problem specified by
H = eye(3);
Aeq = J;
beq = 1 - dJ*dq;
%first coordinate blocked case:
% Aeq1 = [1,0,0];
% beq1 = 0;
% Aeq = [Aeq;Aeq1];
% beq = [beq;beq1];

ddq3 = quadprog(H, [], [], [], Aeq, beq);

tau = M*ddq3 + b + g;
disp("Ex 2.2 tau =")
disp(tau)

%% question 2.3

% Option A --> quadprog
H = M'*M;
f = M'*(b+g);  %i think it is 0.5*tau'*M*ddq
Aeq = J;
beq = 1 - dJ*dq;
ddq = quadprog(H,f, [],[],Aeq,beq);
tau = M*ddq + b + g;
disp("Ex 2.3 tau =")
disp(tau)

% Option B --> null space and pseudoinverse
A1 = [M,-eye(3)];
b1 = [-g-b];
A2 = [J,[0,0,0]];
b2 = [1-dJ*dq];
% keeping orientation and horizontal position case
% J = I_Jp_E(q);
% dJ = I_dJp_E(q,dq);
% A2 = [[I_Jp_E(q),];zeros(3,3)];
% b2 = [[0,1,0]-dJ*dq];
A3 = [zeros(3,3),eye(3)];
b3 = zeros(3,1);    

N = null([A1;A2]);
x = pinv([A1;A2])*[b1;b2]+N*pinv(A3*N)*(b3-A3*pinv([A1;A2])*[b1;b2]);

tau3 = x(4:6);
ddq3 = x(1:3);