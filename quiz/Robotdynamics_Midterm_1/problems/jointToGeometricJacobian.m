function I_J = jointToGeometricJacobian(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  % Get the homogeneous transforms between each pair of coordinate frames.
  TI1 = [cos(q1), -sin(q1), 0,     l0;
         sin(q1),  cos(q1), 0,     0;
              0,       0, 1, 0;
              0,       0, 0,     1];
  T12 = [cos(q2), -sin(q2), 0,     l1;
         sin(q2),  cos(q2), 0,     0;
              0,       0, 1, 0;
              0,       0, 0,     1];  
  T23 = [cos(q3), -sin(q3), 0,     l2;
         sin(q3),  cos(q3), 0,     0;
              0,       0, 1, 0;
              0,       0, 0,     1];  
  T3E = eye(4); T3E(1,4) = l3;

  TI2 = TI1*T12;
  TI3 = TI2*T23;
  
  R_I1 = TI1(1:3,1:3);
  R_I2 = TI2(1:3,1:3);
  R_I3 = TI3(1:3,1:3);
  
  r_I_I1 = TI1(1:3,4);
  r_I_I2 = TI2(1:3,4);
  r_I_I3 = TI3(1:3,4);
  
  n_1 = [0 0 1]';
  n_2 = [0 0 1]';
  n_3 = [0 0 1]';
  
  T_IE = jointToEndeffectorPose( q, params );
  r_I_IE = T_IE(1:3,4);
  % Implement your solution here.
  I_Jp = [   cross(R_I1*n_1, r_I_IE - r_I_I1) ...
            cross(R_I2*n_2, r_I_IE - r_I_I2) ...
            cross(R_I3*n_3, r_I_IE - r_I_I3) ...
        ];
  I_Jr = [   R_I1*n_1 ...
            R_I2*n_2 ...
            R_I3*n_3 ...
        ];

  I_J = [I_Jp;
         I_Jr];
end