function [ T_IF ] = jointToEndeffectorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;
  l4 = params.l4;
  l5 = params.l5;
  L = params.L;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
    
  % Implement your solution here ...
  TI0 = getTransformI0_solution(L);
T01 = getTransform01_solution(l0, l1,q1);
T12 = getTransform12_solution(l2,q2);
T23 = getTransform23_solution(l3,l4,q3);
  T3F = getTransform3F_solution(l5);
  T_IF = TI0*T01*T12*T23*T3F; 
  
end