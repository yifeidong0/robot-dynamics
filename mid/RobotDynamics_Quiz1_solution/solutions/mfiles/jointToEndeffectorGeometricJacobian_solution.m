function I_J = jointToEndeffectorGeometricJacobian_solution(q, params)

  T_I0 = T_I0_solution(q, params);
  T_01 = T_01_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_23 = T_23_solution(q, params);
  T_3F = T_3F_solution(q, params);
  
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  T_IF = T_I3*T_3F;
  
  I_r_I1 = T_I1(1:3,4);
  I_r_I2 = T_I2(1:3,4);
  I_r_I3 = T_I3(1:3,4);
  I_r_IF = T_IF(1:3,4);
  
  I_n_1 = [1;0;0];
  I_n_2 = [1;0;0];
  I_n_3 = [1;0;0];
  
  I_Jp = [cross(I_n_1,I_r_IF-I_r_I1) cross(I_n_2,I_r_IF-I_r_I2) cross(I_n_3,I_r_IF-I_r_I3)];
  I_Jr = [I_n_1 I_n_2 I_n_3];

  I_J = [I_Jp; I_Jr];
end