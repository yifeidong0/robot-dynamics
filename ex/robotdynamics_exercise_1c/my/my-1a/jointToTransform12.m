function T12 = jointToTransform12(q)
    T12 = zeros(4);
    T12(1,1)=cos(q(2,1));    T12(1,3)=sin(q(2,1));
    T12(2,2)=1;
    T12(3,1)=-sin(q(2,1));    T12(3,3)=cos(q(2,1));    T12(3,4)=0.145;
    T12(4,4)=1;
 
end