function T45 = jointToTransform45(q)
    T45 = zeros(4);
    T45(1,1)=cos(q(5));    T45(1,3)=sin(q(5));
    T45(2,2)=1;
    T45(3,1)=-sin(q(5));    T45(3,3)=cos(q(5));    T45(1,4)=0.168;   
    T45(4,4)=1;
 
end