function I_J = jointToGeometricJacobian_solution(q, params)
  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);

  T_I0 = T_I0_solution(q, params);
  T_01 = T_01_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_23 = T_23_solution(q, params);
  T_3E = T_3E_solution(q, params);

  T_IE = T_I0*T_01*T_12*T_23*T_3E;
  
  I_n_1 = [0;0;1];
  I_n_2 = [0;0;1];
  I_n_3 = [0;0;1];
  
  I_r_IE = T_IE(1:3,4);
  
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  
  I_r_I1 = T_I1(1:3,4);
  I_r_I2 = T_I2(1:3,4);
  I_r_I3 = T_I3(1:3,4);
  
  I_Jp = [cross(I_n_1,I_r_IE-I_r_I1) cross(I_n_2,I_r_IE-I_r_I2) cross(I_n_3,I_r_IE-I_r_I3)];
  I_Jr = [I_n_1 I_n_2 I_n_3];

  I_J = [I_Jp; I_Jr];
end