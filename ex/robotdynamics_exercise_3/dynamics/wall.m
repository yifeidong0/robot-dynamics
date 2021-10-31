function F_EE = wall(model, x, x_wall)
% wall is in x-plane with offset x_wall

%	q  : [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
q = x(1:10);
%	qd : [10X1] Generalized velocities
qd = x(11:20);

params = model.parameters.values;
I_J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian
I_T_EE = model.body(11).kinematics.compute.T_IBi(q, qd, [], params);
x_EE = I_T_EE(1, 4);
v_EE = I_J_EE(1,:)*qd;

K = 1e3;
D = 2*sqrt(K);

if x_EE > x_wall
    F_EE = [K*(x_wall-x_EE) - D*v_EE; 0; 0];
else
    F_EE = zeros(3,1);
end

end

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end
