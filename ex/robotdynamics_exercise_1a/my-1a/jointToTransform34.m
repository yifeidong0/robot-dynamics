function T34 = jointToTransform34(q)
    t=q(4);
    T34(1:4,1)=[1,0,0,0]';
    T34(1:4,2)=[0,cos(t),sin(t),0]';
    T34(1:4,3)=[0,-sin(t),cos(t),0]';
    T34(1:4,4)=[0.134,0,0.070,1]';

end