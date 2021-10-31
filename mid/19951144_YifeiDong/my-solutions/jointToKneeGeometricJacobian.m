function I_J = jointToKneeGeometricJacobian(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Get the homogeneous transforms between each pair of coordinate frames.
  T_I0 = T_I0_solution(q, params);
  T_01 = T_01_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_2K = T_2K_solution(q, params);

   T_IK = T_I0*T_01*T_12*T_2K;
  
  I_n_1 = [1;0;0];
  I_n_2 = [1;0;0];

  
  I_r_IK = T_IK(1:3,4);
  
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  
  I_r_I1 = T_I1(1:3,4);
  I_r_I2 = T_I2(1:3,4);
  % Implement your solution here.
  I_Jp = [cross(I_n_1,I_r_IK-I_r_I1) cross(I_n_2,I_r_IK-I_r_I2) ];
  I_Jr = [I_n_1 I_n_2 ];

  I_J = [I_Jp; I_Jr];
end

