function I_J = jointToKneeGeometricJacobian_solution(q, params)
  T_I0 = T_I0_solution(q, params);
  T_01 = T_01_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_2K = T_2K_solution(q, params);
  
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  T_IK = T_I2*T_2K;
  
  I_r_I1 = T_I1(1:3,4);
  I_r_I2 = T_I2(1:3,4);
  I_r_IK = T_IK(1:3,4);
  
  I_n_1 = [1;0;0];
  I_n_2 = [1;0;0];
  
  % Last column should be all zeros since the last joint does not affect
  % the knee velocity 
  I_Jp = [cross(I_n_1,I_r_IK-I_r_I1) cross(I_n_2,I_r_IK-I_r_I2) zeros(3,1)];
  I_Jr = [I_n_1 I_n_2 zeros(3,1)];

  I_J = [I_Jp; I_Jr];
       
end