function T56 = jointToTransform56(q)
    t=q(6);
    T56(1:4,1)=[1,0,0,0]';
    T56(1:4,2)=[0,cos(t),sin(t),0]';
    T56(1:4,3)=[0,-sin(t),cos(t),0]';
    T56(1:4,4)=[0.072,0,0,1]';

end