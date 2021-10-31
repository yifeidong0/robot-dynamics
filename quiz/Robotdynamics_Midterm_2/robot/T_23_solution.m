function T_23 = T_23_solution(q, params)

  T_23 = [cos(q(3)) -sin(q(3)) 0  params.l2;
          sin(q(3))  cos(q(3)) 0  0;
          0          0         1  0;
          0          0         0  1];

end
