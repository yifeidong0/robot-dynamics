function I_J = jointToKneeGeometricJacobian(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Get the homogeneous transforms between each pair of coordinate frames.
  T_I0 = T_I0_solution(q, params);
  T_01 = T_01_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_2K = T_2K_solution(q, params);

  % Implement your solution here.
  I_Jp = [];
  I_Jr = [];

  I_J = [I_Jp;
         I_Jr];
end