function [ q ] = inverse_kinematics_solution( x_E_des, y_E_des, alpha_des, q_0, tol, params )
  % x_E_des: 1x1 desired x position of end-effector
  % y_E_des: 1x1 desired y position of end-effector
  % alpha_des: 1x1 desired alpha angle of end-effector
  % q_0: 3x1 initial guess for joint angles
  % tol: 1x1 tolerance to use as termination criterion
  % params: a struct of parameters

  % Initialize.
  q = q_0;
  chi_err = inf;
  chi_des = [x_E_des; y_E_des; alpha_des];
  
  while (norm(chi_err) > tol)
   T_I0 = T_I0_solution(q, params);
   T_01 = T_01_solution(q, params);
   T_12 = T_12_solution(q, params);
   T_23 = T_23_solution(q, params);
   T_3E = T_3E_solution(q, params);
   T_IE = T_I0*T_01*T_12*T_23*T_3E;

   chi_meas = [T_IE(1:2,4); sum(q)];
   chi_err = chi_des - chi_meas;
   
   Ja = jointToAnalyticJacobian_solution(q, params);
   Ja_psinv = pseudoInverseMat_solution(Ja, 0.01);
   
   deltaQ = Ja_psinv*chi_err;
   q = q + deltaQ;
  end
  
end