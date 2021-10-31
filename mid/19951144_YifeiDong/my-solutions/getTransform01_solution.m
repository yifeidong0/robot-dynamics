    
function T01 = getTransform01_solution(l0, l1,q1)
 
  T01 = [   cos(q1) -sin(q1) 0 0;
            sin(q1) cos(q1) 0 l0;
            0 0 1 l1;
            0 0 0 1];
end