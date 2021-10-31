function T12 = getTransform12_solution(l2,q2)
 
  T12 = [   cos(q2) -sin(q2) 0 l2;
            sin(q2) cos(q2) 0 0;
            0 0 1 0;
            0 0 0 1];
end