    
function TI0 = getTransformI0_solution(L)
 
  TI0 = [   0 0 1 0;
            0 1 0 0;
            -1 0 0 L;
            0 0 0 1];
end