function T01 = jointToTransform01(q)
  if (length(q)>1)
      q = q(1);
  end    
    t=q(1);
    T01(1:4,1)=[cos(t),sin(t),0,0]';
    T01(1:4,2)=[-sin(t),cos(t),0,0]';
    T01(1:4,3)=[0,0,1,0]';
    T01(1:4,4)=[0,0,0.145,1]';

end