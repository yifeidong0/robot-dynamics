function [ T_IE ] = jointToEndeffectorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  
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
          
  T_IE = TI1*T12*T23*T3E; % implement your solution here
end