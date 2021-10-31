function T23 = getTransform23_solution(l3,l4,q3)
 
  T23 = [   cos(q3) -sin(q3) 0 l4;
            sin(q3) cos(q3) 0 l3;
            0 0 1 0;
            0 0 0 1];
end