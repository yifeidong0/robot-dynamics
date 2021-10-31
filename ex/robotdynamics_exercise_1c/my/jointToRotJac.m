function J_R = jointToRotJac(q) 
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    
    C_I1 = T01(1:3,1:3);
    C_I2 = C_I1*T12(1:3,1:3);
    C_I3 = C_I2*T23(1:3,1:3);
    C_I4 = C_I3*T34(1:3,1:3);
    C_I5 = C_I4*T45(1:3,1:3);
    %C_I6 = C_I5*T56(1:3,1:3);
    
    n1 = [0 0 1]';
    n2 = C_I1*[0 1 0]';
    n3 = C_I2*[0 1 0]';
    n4 = C_I3*[1 0 0]';
    n5 = C_I4*[0 1 0]';
    n6 = C_I5*[1 0 0]';

    J_R=[n1,n2,n3,n4,n5,n6];
end