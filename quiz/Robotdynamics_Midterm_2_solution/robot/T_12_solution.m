function T_12 = T_12_solution(q, params)

  T_12 = [cos(q(2)) -sin(q(2)) 0  params.l1;
          sin(q(2))  cos(q(2)) 0  0;
          0          0         1  0;
          0          0         0  1];

end
