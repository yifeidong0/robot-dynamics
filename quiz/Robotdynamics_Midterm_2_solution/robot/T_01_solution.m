function T_01 = T_01_solution(q, params)

  T_01 = [cos(q(1)) -sin(q(1)) 0  0;
          sin(q(1))  cos(q(1)) 0  0;
          0          0         1  0;
          0          0         0  1];

end
